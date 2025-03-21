from setuptools import setup
import os
from glob import glob

package_name = 'multi_drone_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files - explicitly listed
        (os.path.join('share', package_name, 'launch'),
         ['launch/tf_viz.launch.py']),
        # RViz files
        (os.path.join('share', package_name, 'rviz'),
         glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer=' ',
    maintainer_email=' ',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_visualizer = multi_drone_system.tf_visualizer:main',
        ],
    },
)
