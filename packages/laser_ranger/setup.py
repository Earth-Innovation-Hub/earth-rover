from glob import glob
import os

from setuptools import setup

package_name = 'laser_ranger'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jdas',
    maintainer_email='jdas@todo.todo',
    description='USB serial laser ranger publisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laser_ranger_node = laser_ranger.laser_ranger_node:main',
        ],
    },
)
