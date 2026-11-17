"""Launch the Ouster os_driver lifecycle node and move it to the active state."""

import os

import launch
import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('gem_vehicle_sensor_kit_launch'),
        'config', 'ouster', 'driver_params.yaml')

    params_arg = DeclareLaunchArgument('ouster_params_file', default_value=default_params)
    # The driver cloud is in the ouster_ros layout, not the Autoware one; it is converted by
    # gem_ouster_pointcloud_converter before preprocessing.
    points_topic_arg = DeclareLaunchArgument('points_topic', default_value='pointcloud_raw')

    os_driver = LifecycleNode(
        package='ouster_ros',
        executable='os_driver',
        name='os_driver',
        namespace='',
        parameters=[LaunchConfiguration('ouster_params_file')],
        remappings=[('points', LaunchConfiguration('points_topic'))],
        output='screen',
    )

    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(os_driver),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=os_driver, goal_state='inactive',
            entities=[
                LogInfo(msg='os_driver activating...'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(os_driver),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
            handle_once=True,
        )
    )

    finalized = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=os_driver, goal_state='finalized',
            entities=[
                LogInfo(msg='Failed to communicate with the Ouster sensor in a timely manner.'),
                EmitEvent(event=launch.events.Shutdown(reason="Couldn't communicate with sensor")),
            ],
        )
    )

    return launch.LaunchDescription([params_arg, points_topic_arg, os_driver, configure, activate, finalized])
