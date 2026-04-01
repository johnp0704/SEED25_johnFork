from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gui_waypoint_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='airlab',
    maintainer_email='niels.keller1@gmail.com',
    description='GUI-based waypoint navigation with dual camera feeds and dead-reckoning support',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_gui_node = gui_waypoint_nav.waypoint_gui_node:main',
            'waypoint_manager_node = gui_waypoint_nav.waypoint_manager_node:main',
            'path_follower_node = gui_waypoint_nav.path_follower_node:main',
            'dual_camera_node = gui_waypoint_nav.dual_camera_node:main',
            'open_loop_odometry_node = gui_waypoint_nav.open_loop_odometry_node:main',
            'calibration_node = gui_waypoint_nav.calibration_node:main',
        ],
    },
)
