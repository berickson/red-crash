from setuptools import find_packages, setup
from setuptools.command.install import install
from glob import glob
import os
import shutil

package_name = 'joy_soundboard_ros'

# Custom install command to clean sounds directory before installing
class CustomInstall(install):
    def run(self):
        # Clean the sounds directory in install location before installing
        install_base = self.install_base if hasattr(self, 'install_base') else self.prefix
        if install_base:
            sounds_install_path = os.path.join(
                install_base, 'share', package_name, 'sounds'
            )
            if os.path.exists(sounds_install_path):
                shutil.rmtree(sounds_install_path)
        install.run(self)

# Collect all sound files recursively, maintaining directory structure
def get_sound_files():
    data_files = []
    sounds_dir = 'sounds'
    if os.path.exists(sounds_dir):
        for root, dirs, files in os.walk(sounds_dir):
            # Only process directories that have files
            regular_files = [f for f in files if os.path.isfile(os.path.join(root, f))]
            if regular_files:
                # Get relative path from current directory
                rel_path = os.path.relpath(root, '.')
                install_path = 'share/' + package_name + '/' + rel_path
                # Build full paths to the files
                file_paths = [os.path.join(root, f) for f in regular_files]
                data_files.append((install_path, file_paths))
    return data_files

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    cmdclass={
        'install': CustomInstall,
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + get_sound_files(),
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
