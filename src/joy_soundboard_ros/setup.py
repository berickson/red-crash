from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'joy_soundboard_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all sound files
        ('share/' + package_name + '/sounds', glob('sounds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Plays sound files when joystick buttons are pressed',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_soundboard = joy_soundboard_ros.joy_soundboard:main',
        ],
    },
)
