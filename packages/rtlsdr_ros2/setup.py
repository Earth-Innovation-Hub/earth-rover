from setuptools import find_packages, setup

package_name = 'rtlsdr_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rtlsdr_launch.py', 'launch/gps_launch.py', 'launch/gps_map_launch.py']),
        ('share/' + package_name + '/config', ['config/rtlsdr_config.yaml', 'config/gps_config.yaml']),
    ],
    install_requires=['setuptools', 'pyrtlsdr', 'numpy'],
    zip_safe=True,
    maintainer='jdas',
    maintainer_email='jdas@todo.todo',
    description='ROS2 package for RTL-SDR software defined radio data acquisition and publishing',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtlsdr_reader = rtlsdr_ros2.rtlsdr_reader:main',
            'spectrum_visualizer = rtlsdr_ros2.spectrum_visualizer:main',
            'signal_recorder = rtlsdr_ros2.signal_recorder:main',
            'gps_reader = rtlsdr_ros2.gps_reader:main',
            'gps_monitor = rtlsdr_ros2.gps_monitor:main',
            'gps_position_publisher = rtlsdr_ros2.gps_position_publisher:main',
            'gps_map_visualizer = rtlsdr_ros2.gps_map_visualizer:main',
        ],
    },
)
