from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

pkg_name1 = 'dynamixel_handler'

def generate_launch_description():
    ld = LaunchDescription()

    config1 = os.path.join(
        get_package_share_directory(pkg_name1),
        'config',
        'dynamixel_handler.yaml'
    )

    node1 = Node(
        package=pkg_name1,
        executable='dynamixel_handler_node',
        name='dxl_handler',
        namespace='ns2',
        output='screen',
        emulate_tty=True,
        parameters=[config1]
    )

    ld.add_action(node1)
# 複数ノードを追加する場合は，configN,nodeNを作ってld.add_action(nodeN)?

    return ld
