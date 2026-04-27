from setuptools import setup
import os
from glob import glob

package_name = 'task_dispatcher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('lib', package_name), glob('task_dispatcher/*.py')),
    ],
    install_requires=['setuptools', 'paho-mqtt', 'requests'], # 添加requests库依赖
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Task dispatcher node for ROS2 that subscribes to MQTT messages and calls ROS services',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'task_dispatcher_node = task_dispatcher.task_dispatcher_node:main',
        ],
    },
)