from setuptools import setup
import os
from glob import glob

package_name = 'robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Register package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install URDF files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.xacro')),

        # Install launch files (if any)
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        # Install meshes (if any)
        (os.path.join('share', package_name, 'meshes'),
            glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Robot description package containing URDF/Xacro files',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
