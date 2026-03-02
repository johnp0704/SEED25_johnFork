from setuptools import setup

package_name = 'mv_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_data={'mv_controller': ['*.npz']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        
          'controller = mv_controller.mv_gtg_ros:main',
        ],
    },
)
