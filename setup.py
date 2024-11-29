from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'robot-football'
LAUNCHFILE = "robot_launch.py"
WORLDFILE = "my_world.wbt"
ROBOTURDF = "my_robot.urdf"
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/' + LAUNCHFILE]))
data_files.append(('share/' + package_name + '/worlds', ['worlds/' + WORLDFILE]))
data_files.append(('share/' + package_name + '/resource', ['resource/' + ROBOTURDF]))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harryk',
    maintainer_email='harryk@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
          "console_scripts": [
            'my_robot_driver = robot_football.my_robot_driver:main',
        ],
    },
)
