from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'system_management'

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
            'gui = gui.management_gui:main',
            'VC_supervisor =        safety_controller.voltage_current_supervisor:main',
            'master_supervisor =    safety_controller.master_supervisor:main',
            'camera_supervisor =    safety_controller.camera_supervisor:main',          
            'system_health =        safety_controller.system_health_laptop:main',            
            'esp32_safety =             others.esp32_safety:main',
            'robot_lights_controller =  others.robot_lights_controller:main',
            'test_lazo_abierto = test_code.test_lazo_abierto:main',
            'gui_lazo_cerrado =  test_code.gui_lazo_cerrado:main',
        ],
    },
)
