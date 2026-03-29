from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'pick_and_place'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Pick and place task',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = pick_and_place.camera_node:main',
            'cube_spawner = pick_and_place.cube_spawner:main',
            'pick_middle_cube = pick_and_place.pick_middle_cube:main',
            'rotate_scan_color = pick_and_place.rotate_scan_color:main',
            'detect_small_cubes = pick_and_place.detect_small_cubes:main',
            'puzzle_pick_place = pick_and_place.puzzle_pick_place:main',
        ],
    },
)
