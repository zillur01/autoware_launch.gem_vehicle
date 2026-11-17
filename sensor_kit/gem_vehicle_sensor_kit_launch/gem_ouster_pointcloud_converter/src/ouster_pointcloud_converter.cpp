// Copyright 2026 Zillur Rahman
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Converts the ouster_ros "original" point cloud
//   {x, y, z, intensity(f32), t(u32 ns), reflectivity(u16), ring(u16), ambient(u16), range(u32 mm)}
// into the Autoware PointXYZIRCAEDT layout
//   {x, y, z, intensity(u8), return_type(u8), channel(u16), azimuth, elevation, distance, time_stamp}
// so that the Autoware sensing / perception stack (distortion corrector, ring outlier filter,
// TransFusion, CenterPoint, ...) can consume it.

#include <autoware/point_types/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace gem_ouster_pointcloud_converter
{
using autoware::point_types::PointXYZIRCAEDT;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

namespace
{
constexpr float kTwoPi = 2.0F * static_cast<float>(M_PI);

std::optional<PointField> find_field(const PointCloud2 & msg, const std::string & name)
{
  for (const auto & f : msg.fields) {
    if (f.name == name && f.count == 1) {
      return f;
    }
  }
  return std::nullopt;
}

// Reads a scalar of any PointField datatype as double.
double read_as_double(const uint8_t * p, uint8_t datatype)
{
  switch (datatype) {
    case PointField::INT8: {
      int8_t v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    case PointField::UINT8:
      return *p;
    case PointField::INT16: {
      int16_t v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    case PointField::UINT16: {
      uint16_t v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    case PointField::INT32: {
      int32_t v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    case PointField::UINT32: {
      uint32_t v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    case PointField::FLOAT32: {
      float v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    case PointField::FLOAT64: {
      double v;
      std::memcpy(&v, p, sizeof(v));
      return v;
    }
    default:
      return 0.0;
  }
}

PointField make_field(const std::string & name, uint32_t offset, uint8_t datatype)
{
  PointField f;
  f.name = name;
  f.offset = offset;
  f.datatype = datatype;
  f.count = 1;
  return f;
}

std::vector<PointField> make_xyzircaedt_fields()
{
  return {
    make_field("x", offsetof(PointXYZIRCAEDT, x), PointField::FLOAT32),
    make_field("y", offsetof(PointXYZIRCAEDT, y), PointField::FLOAT32),
    make_field("z", offsetof(PointXYZIRCAEDT, z), PointField::FLOAT32),
    make_field("intensity", offsetof(PointXYZIRCAEDT, intensity), PointField::UINT8),
    make_field("return_type", offsetof(PointXYZIRCAEDT, return_type), PointField::UINT8),
    make_field("channel", offsetof(PointXYZIRCAEDT, channel), PointField::UINT16),
    make_field("azimuth", offsetof(PointXYZIRCAEDT, azimuth), PointField::FLOAT32),
    make_field("elevation", offsetof(PointXYZIRCAEDT, elevation), PointField::FLOAT32),
    make_field("distance", offsetof(PointXYZIRCAEDT, distance), PointField::FLOAT32),
    make_field("time_stamp", offsetof(PointXYZIRCAEDT, time_stamp), PointField::UINT32),
  };
}
}  // namespace

class OusterPointCloudConverter : public rclcpp::Node
{
public:
  explicit OusterPointCloudConverter(const rclcpp::NodeOptions & options)
  : Node("ouster_pointcloud_converter", options), output_fields_(make_xyzircaedt_fields())
  {
    // "reflectivity": calibrated 0-255 target reflectivity (closest to Velodyne/Hesai intensity)
    // "intensity":    raw signal photon count, clamped to 0-255
    intensity_source_ = declare_parameter<std::string>("intensity_source", "reflectivity");
    intensity_scale_ = declare_parameter<double>("intensity_scale", 1.0);
    min_distance_ = declare_parameter<double>("min_distance", 0.0);
    max_distance_ = declare_parameter<double>("max_distance", 1000.0);
    // Empty: keep the sensor frame (sensor kit pipeline). Set e.g. "base_link" to transform x/y/z
    // into that frame (standalone use, e.g. feeding a detector directly from a rosbag).
    // azimuth / elevation / distance always stay sensor-relative, as in the Autoware convention.
    output_frame_ = declare_parameter<std::string>("output_frame", "");

    if (!output_frame_.empty()) {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    }

    if (intensity_source_ != "reflectivity" && intensity_source_ != "intensity") {
      throw std::invalid_argument(
        "intensity_source must be 'reflectivity' or 'intensity', got '" + intensity_source_ + "'");
    }

    pub_ = create_publisher<PointCloud2>("output", rclcpp::SensorDataQoS().keep_last(1));
    sub_ = create_subscription<PointCloud2>(
      "input", rclcpp::SensorDataQoS().keep_last(1),
      [this](const PointCloud2::ConstSharedPtr msg) { on_pointcloud(msg); });

    RCLCPP_INFO(
      get_logger(), "Ouster -> PointXYZIRCAEDT converter started (intensity_source: %s)",
      intensity_source_.c_str());
  }

private:
  void on_pointcloud(const PointCloud2::ConstSharedPtr & in)
  {
    if (pub_->get_subscription_count() == 0 && pub_->get_intra_process_subscription_count() == 0) {
      return;
    }

    const auto fx = find_field(*in, "x");
    const auto fy = find_field(*in, "y");
    const auto fz = find_field(*in, "z");
    const auto fi = find_field(*in, intensity_source_);
    const auto ft = find_field(*in, "t");
    const auto fring = find_field(*in, "ring");
    const auto frange = find_field(*in, "range");

    if (
      !fx || !fy || !fz || fx->datatype != PointField::FLOAT32 ||
      fy->datatype != PointField::FLOAT32 || fz->datatype != PointField::FLOAT32) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Input cloud has no float32 x/y/z fields; dropping.");
      return;
    }
    if (!fi || !ft || !fring) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Input cloud is missing '%s', 't' or 'ring'. Set the Ouster driver point_type to "
        "'original' (or 'native').",
        intensity_source_.c_str());
      return;
    }

    // Sensor mount transform (assumed static, so the latest one is used).
    std::optional<tf2::Transform> to_output_frame;
    if (!output_frame_.empty() && output_frame_ != in->header.frame_id) {
      try {
        const auto tf_msg =
          tf_buffer_->lookupTransform(output_frame_, in->header.frame_id, tf2::TimePointZero);
        tf2::Transform tf;
        tf2::fromMsg(tf_msg.transform, tf);
        to_output_frame = tf;
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Waiting for transform %s -> %s: %s",
          in->header.frame_id.c_str(), output_frame_.c_str(), e.what());
        return;
      }
    }

    const size_t n_in = static_cast<size_t>(in->width) * in->height;
    auto out = std::make_unique<PointCloud2>();
    out->header = in->header;
    if (to_output_frame) {
      out->header.frame_id = output_frame_;
    }
    out->height = 1;
    out->fields = output_fields_;
    out->is_bigendian = false;
    out->point_step = sizeof(PointXYZIRCAEDT);
    out->data.resize(n_in * sizeof(PointXYZIRCAEDT));

    const uint8_t * in_data = in->data.data();
    uint8_t * out_ptr = out->data.data();
    size_t n_out = 0;

    // Row-major: Ouster rows are beams, so the output is grouped by channel and ordered by
    // azimuth within each channel, which is what the ring outlier filter expects.
    for (uint32_t row = 0; row < in->height; ++row) {
      const uint8_t * row_ptr = in_data + static_cast<size_t>(row) * in->row_step;
      for (uint32_t col = 0; col < in->width; ++col) {
        const uint8_t * p = row_ptr + static_cast<size_t>(col) * in->point_step;

        if (frange && read_as_double(p + frange->offset, frange->datatype) == 0.0) {
          continue;  // no return
        }

        float x;
        float y;
        float z;
        std::memcpy(&x, p + fx->offset, sizeof(float));
        std::memcpy(&y, p + fy->offset, sizeof(float));
        std::memcpy(&z, p + fz->offset, sizeof(float));
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }

        const float xy = std::hypot(x, y);
        const float distance = std::hypot(xy, z);
        if (distance <= min_distance_ || distance > max_distance_) {
          continue;
        }

        float azimuth = std::atan2(y, x);
        if (azimuth < 0.0F) {
          azimuth += kTwoPi;
        }

        const double raw_intensity = read_as_double(p + fi->offset, fi->datatype) * intensity_scale_;
        const double t_ns = read_as_double(p + ft->offset, ft->datatype);

        PointXYZIRCAEDT pt;
        if (to_output_frame) {
          const tf2::Vector3 v = (*to_output_frame) * tf2::Vector3(x, y, z);
          pt.x = static_cast<float>(v.x());
          pt.y = static_cast<float>(v.y());
          pt.z = static_cast<float>(v.z());
        } else {
          pt.x = x;
          pt.y = y;
          pt.z = z;
        }
        pt.intensity = static_cast<uint8_t>(std::clamp(raw_intensity, 0.0, 255.0));
        pt.return_type = autoware::point_types::ReturnType::SINGLE_STRONGEST;
        pt.channel = static_cast<uint16_t>(read_as_double(p + fring->offset, fring->datatype));
        pt.azimuth = azimuth;
        pt.elevation = std::atan2(z, xy);
        pt.distance = distance;
        pt.time_stamp = static_cast<uint32_t>(std::max(0.0, t_ns));

        std::memcpy(out_ptr, &pt, sizeof(PointXYZIRCAEDT));
        out_ptr += sizeof(PointXYZIRCAEDT);
        ++n_out;
      }
    }

    out->data.resize(n_out * sizeof(PointXYZIRCAEDT));
    out->width = static_cast<uint32_t>(n_out);
    out->row_step = out->width * out->point_step;
    out->is_dense = true;
    pub_->publish(std::move(out));
  }

  std::string intensity_source_;
  double intensity_scale_;
  double min_distance_;
  double max_distance_;
  std::string output_frame_;
  const std::vector<PointField> output_fields_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
};

}  // namespace gem_ouster_pointcloud_converter

RCLCPP_COMPONENTS_REGISTER_NODE(gem_ouster_pointcloud_converter::OusterPointCloudConverter)
