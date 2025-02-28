<<<<<<< HEAD
from setuptools import setup, find_packages
=======
from setuptools import setup
>>>>>>> 950ca54 (removed gazebo integration)
import os
from glob import glob

package_name = 'multi_drone_system'

setup(
    name=package_name,
    version='0.0.0',
<<<<<<< HEAD
    packages=find_packages(),
=======
    packages=[package_name],
>>>>>>> 950ca54 (removed gazebo integration)
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        # World files
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*.world')),
<<<<<<< HEAD
        # Model files
        (os.path.join('share', package_name, 'models/drone'),
         glob('models/drone/*')),
=======
        # RViz files
        (os.path.join('share', package_name, 'rviz'),
         glob('rviz/*.rviz')),
>>>>>>> 950ca54 (removed gazebo integration)
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jching',
    maintainer_email='jching2@yahoo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
<<<<<<< HEAD
            'drone_node = multi_drone_system.drone_node:main',
=======
            'drone_controller = multi_drone_system.drone_controller:main',
            'drone_swarm_controller = multi_drone_system.drone_controller:main',  # Update this to point to the swarm controller
            'tf_visualizer = multi_drone_system.tf_visualizer:main',
            'gazebo_sync = multi_drone_system.gazebo_sync:main',
            'gazebo_test = multi_drone_system.gazebo_test:main',
>>>>>>> 950ca54 (removed gazebo integration)
        ],
    },
)
