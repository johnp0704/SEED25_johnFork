from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'opti_waypoint_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='airlab',
    maintainer_email='airlab@todo.todo',
    description='OptiTrack waypoint following architecture',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_manager = opti_waypoint_nav.waypoint_manager:main'
            'path_follower = opti_waypoint_nav.path_follower:main'
        ],
    },
)
