
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

import launch
from launch import LaunchDescription
from launch import LaunchIntrospector
from launch import LaunchService

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from launch_ros import get_default_launch_description
import launch_ros.actions


def generate_launch_description():
    ld = LaunchDescription()

    ns = "/image" 

    usb_camera = launch_ros.actions.Node(
            package='usb_camera_driver', node_executable='usb_camera_driver_node', output='screen', node_namespace=ns, 
            parameters=[{"camera_calibration_file": "file:///config/camera.yaml"}])

    ld.add_action(usb_camera)

    return ld


def main(argv=sys.argv[1:]):
    """Main."""
    ld = generate_launch_description()

    print('Starting introspection of launch description...')
    print('')

    print(LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    #ls = LaunchService(debug=True)
    ls = LaunchService()
    ls.include_launch_description(get_default_launch_description())
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main()
