from setuptools import setup

package_name = 'mocap_listener'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
        'rigid_listener = mocap_listener.rigid_listener:main',
        'gtg_node = mocap_listener.gtg_node:main',
        'gtg_node_nk = mocap_listener.gtg_node_nk:main',
        'nk_telem = mocap_listener.nk_telem.py:main',
        'gtg_node_mathAngle = mocap_listener.gtg_node_mathAngle:main',
        'telemetry_node = mocap_listener.telemetry_node:main',
        'plot_robot_and_goal = mocap_listener.plot_robot_and_goal:main',
    ],
    },
)
