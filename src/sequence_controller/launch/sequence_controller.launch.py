# from launch import LaunchDescription
# from launch_ros.actions import Node


# def generate_launch_description():
#     return LaunchDescription(
#         [
#             Node(
#                 package="relbot_simulator",
#                 executable="relbot_simulator",
#                 arguments=["--ros-args", "--log-level", "WARN"],
#             ),
#             Node(
#                 package="cam2image_vm2ros",
#                 executable="cam2image",
#                 arguments=["--ros-args --params-file src/cam2image_vm2ros/config/cam2image_relbot.yaml", "--log-level", "WARN"],
#             ),
#             Node(
#                 package="object_center",
#                 executable="object_center",
#                 #remappings=[("/image", "/output/moving_camera")],
#             ),
#             Node(
#                 package="sequence_controller",
#                 executable="sequence_controller",
#                 remappings=[
#                     ("camera_position", "/output/camera_position"),
#                     ("left_motor_setpoint_vel", "/input/right_motor/setpoint_vel"),
#                     ("right_motor_setpoint_vel", "/input/right_motor/setpoint_vel"),
#                 ],
#             ),
#         ]
#     )
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# for the yaml file
cam2image_pkg_share = get_package_share_directory('cam2image_vm2ros')
cam2image_config_path = os.path.join(cam2image_pkg_share, 'config', 'cam2image_relbot.yaml')

def generate_launch_description():

    # 1. Start the ROS-Xenomai Bridge node
    start_ros_xeno_bridge = Node(
        package='ros_xeno_bridge',
        executable='RosXenoBridge',
        name='ros_xeno_bridge_node',
        output='screen'
    )

    # 2. Start camera node
    start_camera_node = Node(
        package="cam2image_vm2ros",
        executable="cam2image",
        name="cam2image_node",
        parameters=[cam2image_config_path],
        output='screen'
    )

    # 3. Starting the ball tracker node
    start_ball_tracker_node = Node(
        package="ball_tracker",     
        executable="ball_tracker",   
        name="ball_tracker",    
        output='screen',
        # remappings=[
        #     ("/output/moving_camera", "/image")
        # ]
    )


    # 4. Start sequence controller node
    start_sequence_controller_node = Node(
        package="sequence_controller",
        executable="sequence_controller",
        name="sequence_controller_node",
        output='screen',
        remappings=[
            ("light_position", "/light_position"),
            ("left_motor_setpoint_vel", "/Ros2Xeno"),
            ("right_motor_setpoint_vel", "/Ros2Xeno"),
            ("/Xeno2Ros", "/Xeno2Ros"),
        ]
    )

    # 5. Publish the dummy /Ros2Xeno message 
    publish_dummy_ros2xeno = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' topic pub --once /Ros2Xeno xrf2_msgs/msg/Ros2Xeno "{example_a: 0.0, example_b: 0.0}"'
        ]],
        shell=True,
        output='screen'
    )

    # 6. Initialise command
    publish_init_cmd = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' topic pub --once /XenoCmd std_msgs/msg/Int32 "{data: 1}"' 
        ]],
        shell=True,
        output='screen'
    )

    # 7. Run command
    publish_run_cmd = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' topic pub --once /XenoCmd std_msgs/msg/Int32 "{data: 2}"' 
        ]],
        shell=True,
        output='screen'
    )

    ld = LaunchDescription()

    # Add ROS nodes
    ld.add_action(LogInfo(msg="Starting ROS-Xenomai Bridge..."))
    ld.add_action(start_ros_xeno_bridge)

    ld.add_action(LogInfo(msg="Starting Camera Node..."))
    ld.add_action(start_camera_node)

    # # Delay
    # ld.add_action(TimerAction(period=1.0, actions=[
    #     LogInfo(msg="Starting Object Center Node..."),
    #     start_object_center_node
    #     ]))

    ld.add_action(TimerAction(period=1.5, actions=[
        LogInfo(msg="Starting Sequence Controller..."),
        start_sequence_controller_node
        ]))

    # Add timed command publications 
    ld.add_action(TimerAction(period=2.0, actions=[
        LogInfo(msg="Publishing dummy message to /Ros2Xeno..."),
        publish_dummy_ros2xeno
    ]))

    ld.add_action(TimerAction(period=3.0, actions=[
        LogInfo(msg="Publishing Initialise command to /XenoCmd..."),
        publish_init_cmd
    ]))

    ld.add_action(TimerAction(period=4.0, actions=[
        LogInfo(msg="Publishing Run command to /XenoCmd..."),
        publish_run_cmd
    ]))

    return ld