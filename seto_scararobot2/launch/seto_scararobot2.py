import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

import launch  # noqa: E402
import launch.actions  # noqa: E402
import launch.events  # noqa: E402

import launch_ros.actions  # noqa: E402
import launch_ros.events  # noqa: E402
import launch_ros.events.lifecycle  # noqa: E402

import lifecycle_msgs.msg  # noqa: E402


def generate_launch_description():
    # Get the launch directory
    urdf_dir = get_package_share_directory('seto_scararobot2_description')
    launch_dir = os.path.join(urdf_dir, 'launch')


    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    urdf_file= LaunchConfiguration('urdf_file')
    use_seto_scararobot2 = LaunchConfiguration('use_seto_scararobot2')
    use_move_arm = LaunchConfiguration('use_move_arm')
    use_beads_setter = LaunchConfiguration('use_beads_setter')
    use_beadsee_dummy = LaunchConfiguration('use_beadsee_dummy')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(urdf_dir, 'rviz', 'seto_scararobot2.rviz'),
        description='Full path to the RVIZ config file to use')  
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
    declare_urdf_cmd = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(urdf_dir, 'urdf', 'seto_scararobot2.urdf'),
        description='Whether to start RVIZ')
    declare_use_seto_scararobot2_cmd = DeclareLaunchArgument(
        'use_seto_scararobot2',
        default_value='True',
        description='Whether to start seto_scararobot2')
    declare_use_move_arm_cmd = DeclareLaunchArgument(
        'use_move_arm',
        default_value='True',
        description='Whether to start move_arm')
    declare_use_beads_setter_cmd = DeclareLaunchArgument(
        'use_beads_setter',
        default_value='True',
        description='Whether to start beads_setter')
    declare_use_beadsee_dummy_cmd = DeclareLaunchArgument(
        'use_beadsee_dummy',
        default_value='True',
        description='Whether to start beadsee_dummy')

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf_file])
    
    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')
    
    start_seto_scararobot2_cmd = LifecycleNode(
        condition=IfCondition(use_seto_scararobot2),
        package='seto_scararobot2',
        executable='seto_scararobot2',
        name='seto_scararobot2',
        namespace='',
        output='screen')
    
    start_move_arm_cmd = Node(
        condition=IfCondition(use_move_arm),
        package='seto_scararobot2',
        executable='move_arm',
        name='move_arm')

    start_beads_setter_cmd = Node(
        condition=IfCondition(use_beads_setter),
        package='beads_setter',
        executable='beads_setter',
        name='beads_setter')
    
    start_beadsee_dummy_cmd = Node(
        condition=IfCondition(use_beadsee_dummy),
        package='setoros_BeadsEE',
        executable='beadsee_dummy',
        name='beadsee_dummy')

    # Create the launch description and populate
    ld = LaunchDescription()

    # When the seto_scararobot2 reaches the 'inactive' state, make it take the 'activate' transition.
    register_event_handler_for_seto_scararobot2_reaches_inactive_state = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=start_seto_scararobot2_cmd, goal_state='inactive',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'seto_scararobot2' reached the 'inactive' state, 'activating'."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(start_seto_scararobot2_cmd),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # When the seto_scararobot2 node reaches the 'active' state, log a message and start the listener node.
    register_event_handler_for_seto_scararobot2_reaches_active_state = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=start_seto_scararobot2_cmd, goal_state='active',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'seto_scararobot2' reached the 'active' state."),
            ],
        )
    )

    # Make the seto_scararobot2 node take the 'configure' transition.
    emit_event_to_request_that_seto_scararobot2_does_configure_transition = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(start_seto_scararobot2_cmd),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Declare the launch options
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_urdf_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_seto_scararobot2_cmd)
    ld.add_action(declare_use_move_arm_cmd)
    ld.add_action(declare_use_beads_setter_cmd)
    ld.add_action(declare_use_beadsee_dummy_cmd)

    # Add any conditioned actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(start_seto_scararobot2_cmd)
    ld.add_action(start_move_arm_cmd)
    ld.add_action(start_beads_setter_cmd)
    ld.add_action(register_event_handler_for_seto_scararobot2_reaches_inactive_state)
    ld.add_action(register_event_handler_for_seto_scararobot2_reaches_active_state)
    ld.add_action(emit_event_to_request_that_seto_scararobot2_does_configure_transition)
    ld.add_action(start_beadsee_dummy_cmd)
    return ld   
    
