from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'saves'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raynel',
    maintainer_email='luchitov2001@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ekf_recorder = saves.ekf_recorder:main',
            'data_player = saves.data_player:main',
            'data_recorder = saves.data_recorder:main',
            'logging_manager = saves.logging_manager:main',
            'perception_recorder = saves.perception_recorder:main',
            'healthy_recorder = saves.healthy_recorder:main',
            'perception_recorder_left = saves.perception_recorder_left:main',
            'perception_recorder_right = saves.perception_recorder_right:main',
            'control_signals = saves.control_signals:main',
            'image_mode = saves.image_mode:main',
            'image_publisher = saves.image_publisher:main',
        ],
    },
)
