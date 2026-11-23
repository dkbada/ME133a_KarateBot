"""G1 holding the sword with both hands, doing a vertical swing.

   ros2 launch karate swing.launch.py

This starts
  1) RVIZ, ready to view the robot
  2) The robot_state_publisher to broadcast the robot model
  3) The swing node, publishing the joint states
"""

import os

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                            import LaunchDescription
from launch.actions                    import Shutdown
from launch_ros.actions                import Node


def generate_launch_description():
    # Locate the RVIZ configuration and URDF files.
    rvizcfg = os.path.join(pkgdir('utils'),  'rviz', 'viewurdfplus.rviz')
    urdf    = os.path.join(pkgdir('karate'), 'urdf', 'g1_23dof_sword.urdf')

    # Load the robot's URDF file (XML).
    with open(urdf, 'r') as file:
        robot_description = file.read()

    node_robot_state_publisher = Node(
        name       = 'robot_state_publisher',
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description': robot_description}])

    node_rviz = Node(
        name       = 'rviz',
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rvizcfg],
        on_exit    = Shutdown())

    node_swing = Node(
        name       = 'swing',
        package    = 'karate',
        executable = 'swing',
        output     = 'screen')

    return LaunchDescription([
        node_robot_state_publisher,
        node_rviz,
        node_swing,
    ])
