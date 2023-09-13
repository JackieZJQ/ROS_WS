from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    camOpennode = Node(
        package="cam_module",
        node_namespace="shortCam",
        node_executable="camOpen"
    )

    laneDetectnode = Node(
        package="cam_module",
        node_namespace="camLane",
        node_executable="laneDetect"
    )

    launch_d = LaunchDescription([camOpennode, laneDetectnode])

    return launch_d