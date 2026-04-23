from setuptools import setup
import os
from glob import glob

package_name = 'demo_night'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='airlab',
    maintainer_email='niels.keller1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'commander_node = demo_night.commander_node:main',
            'gui_node = demo_night.gui_node:main',
            'aruco_rehoming_node = demo_night.aruco_rehoming_node:main',
            'optical_path_follower_node = demo_night.optical_path_follower_node:main',
            'gtg_controller_node = demo_night.gtg_controller_node:main',
            'realsense_node = demo_night.realsense_node:main',
            'display_node = demo_night:display_node:main',
        ],
    },
)
