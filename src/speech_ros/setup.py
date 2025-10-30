from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'speech_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install secrets folder structure
        ('share/' + package_name + '/secrets', ['secrets/README.md', 'secrets/.gitignore']),
        # Install voice models for Piper TTS
        ('share/' + package_name + '/voices', glob('voices/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Speech recognition and text-to-speech for ROS2',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech = speech_ros.speech:main',
            'command_interpreter = speech_ros.command_interpreter:main',
        ],
    },
)
