from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'path_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tu_nombre',
    maintainer_email='tu@email.com',
    description='3D Path Planner para UAV con A* sobre voxel grid',
    license='GPL-3.0',
    entry_points={
        'console_scripts': [
            'planner_node   = path_planner.planner_node:main',
            'mavros_mission = path_planner.mavros_mission:main',
        ],
    },
)