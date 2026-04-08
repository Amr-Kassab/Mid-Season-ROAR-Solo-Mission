from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        # Install RViz config files  <-- ADD THIS LINE
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Amr Kassab',
    maintainer_email='amrkassab2005@example.com',
    description='Fake SLAM and Robot Position Updater',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # format: 'executable_name = package_name.file_name:main_function'
            'fake_slam = mission.fake_slam:main',
            'robot_position_updater = mission.robot_position_updater:main',
            'planner = mission.planner:main',         
            'controller = mission.controller:main',
        ],
    },
)