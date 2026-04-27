from setuptools import setup
import os
from glob import glob

package_name = 'multi_algo_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, package_name + '.algos', package_name + '.srv'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('lib', package_name), glob('multi_algo_manager/*.py')),
        # 使用递归方式包含所有模型文件和目录
        (os.path.join('share', package_name, 'models'),
         ['models/README.md']),
        (os.path.join('share', package_name, 'models', 'trash_detect', 'pytorch_model'),
         glob('models/trash_detect/pytorch_model/*')),
        (os.path.join('share', package_name, 'models', 'trash_detect', 'best_rknn_model_trash'),
         glob('models/trash_detect/best_rknn_model_trash/*')),
        (os.path.join('share', package_name, 'models', 'device_check', 'pytorch_model'),
         glob('models/device_check/pytorch_model/*')),
        (os.path.join('share', package_name, 'models', 'device_check', 'best_rknn_model_device'),
         glob('models/device_check/best_rknn_model_device/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Multi-algorithm manager demo for ROS2 with service control',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'algo_manager_node = multi_algo_manager.algo_manager_node:main',
            'trash_detect_node = multi_algo_manager.algos.trash_detect:main',
            'device_check_node = multi_algo_manager.algos.device_check:main',
            'channel_monitor_node = multi_algo_manager.algos.channel_monitor:main',
            'line_integrity_node = multi_algo_manager.algos.line_integrity:main',
        ],
    },
)
