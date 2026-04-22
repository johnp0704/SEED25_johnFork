from setuptools import setup
import os
from glob import glob

package_name = 'ml_red_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
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
        'ml_red_detector_node  = ml_red_controller.ml_red_detector_node:main',
        'arducam_detector_node = ml_red_controller.arducam_detector_node:main',
        'gtg_controller_node   = ml_red_controller.gtg_controller_node:main',
        'gtg_motor_node        = ml_red_controller.gtg_motor_node:main',
        'cam_display_node      = ml_red_controller.cam_display_node:main',
        ],
    },
)
