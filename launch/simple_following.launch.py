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

    car_controller = LaunchDescription(
        [
            launch_ros.actions.Node(
                package="stupid_car", executable="car", output="screen"
            )
        ]
    )

    ld = LaunchDescription()

    ld.add_action(uwb_driver)
    ld.add_action(car_controller)

    return ld
