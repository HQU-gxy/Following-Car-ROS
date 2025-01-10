from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    uwb_driver = LaunchDescription(
        [
            launch_ros.actions.Node(
                package="stupid_car", executable="uwb", output="screen"
            )
        ]
    )

    pose_publisher = LaunchDescription(
        [
            launch_ros.actions.Node(
                package="stupid_car", executable="uwb2GaolPose", output="screen"
            )
        ]
    )

    base_to_uwb_module = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_uwb_module",
        arguments=["0.52", "0", "0", "0", "0", "0", "base_footprint", "uwb"],
    )

    ld = LaunchDescription()

    ld.add_action(uwb_driver)
    ld.add_action(pose_publisher)
    ld.add_action(base_to_uwb_module)

    return ld
