import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    """The launch description for the package."""
    package_dir = get_package_share_directory("robot_football")
    robot_description_path = os.path.join(package_dir, "resource", "my_robot.urdf")

    webots = WebotsLauncher(
        world=os.path.join(package_dir, "worlds", "my_world.wbt"),
        ros2_supervisor=True,
        mode="realtime",
    )
    # print(robot_description_path)

    my_robot_driver = WebotsController(
        robot_name="my_robot",
        parameters=[
            {"robot_description": robot_description_path},
        ],
        respawn=True,
    )
    print(my_robot_driver)

    print(f"WEBOTS NODEPACKAGE = {webots._supervisor.describe()}")

    # Declare the reset handler that respawns nodes when robot_driver exits

    # reset_handler = launch.actions.RegisterEventHandler(
    #     event_handler=launch.event_handlers.OnProcessExit(
    #         target_action=my_robot_driver,
    #         on_exit=get_ros2_control_spawners,
    #     )
    # )
    # obstacle_avoider = Node(
    #     package="robot-football",
    #     executable="obstacleavoider",
    # )
    # ballSupervisor = Node(
    #     package="robot-football",
    #     executable="ballSupervisor",
    # )

    return LaunchDescription(
        [
            webots,
            webots._supervisor,
            my_robot_driver,
            # obstacle_avoider,
            # reset_handler,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
        # + get_ros2_control_spawners()
    )
