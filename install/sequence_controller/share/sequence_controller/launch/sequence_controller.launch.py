from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="relbot_simulator",
                executable="relbot_simulator",
                arguments=["--ros-args", "--log-level", "WARN"],
            ),
            Node(
                package="cam2image_vm2ros",
                executable="cam2image",
                arguments=["--ros-args --params-file src/cam2image_vm2ros/config/cam2image_relbot.yaml", "--log-level", "WARN"],
            ),
            Node(
                package="object_center",
                executable="object_center",
                #remappings=[("/image", "/output/moving_camera")],
            ),
            Node(
                package="sequence_controller",
                executable="sequence_controller",
                remappings=[
                    ("camera_position", "/output/camera_position"),
                    ("left_motor_setpoint_vel", "/input/right_motor/setpoint_vel"),
                    ("right_motor_setpoint_vel", "/input/right_motor/setpoint_vel"),
                ],
            ),
        ]
    )
