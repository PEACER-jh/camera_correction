import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node, SetParameter, PushRosNamespace
from launch.actions import TimerAction, Shutdown
from launch import LaunchDescription

sys.path.sys.path.append(os.path.join(os.getcwd(), 'rmos_bringup', 'launch'))
def get_params(name):
    return os.path.join(os.getcwd(), 'rmos_bringup', 'config', 'node_params', '{}_params.yaml'.format(name))

def generate_launch_description():
    daheng_node_  = Node(
            package='rmos_camera',
            executable='daheng_camera',
            output='screen',
            parameters=[get_params('camera')],
            # extra_arguments=[{'use_intra_process_comms': True}]
        )