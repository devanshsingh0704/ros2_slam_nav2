from setuptools import setup
import os
from glob import glob

package_name = 'robot_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        # Install world files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),

        # Install models (IMPORTANT FIX)
        (os.path.join('share', package_name, 'models', 'maze_1'),
            glob('models/maze_1/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Gazebo worlds and models',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
