from setuptools import setup
import os
from glob import glob

package_name = 'ryugu_sim'


def package_files(data_files, directories):
    """Recursively install every file under each directory, preserving structure.
    Needed because worlds/ and models/ were previously omitted from data_files,
    so colcon build never synced edits to install/ (silent stale-copy bug)."""
    paths_by_dest = {}
    for directory in directories:
        for path, _, filenames in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                dest = os.path.join('share', package_name, path)
                paths_by_dest.setdefault(dest, []).append(file_path)
    for dest, files in paths_by_dest.items():
        data_files.append((dest, files))
    return data_files


setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=package_files([
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.py')),
    ], ['worlds', 'models']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Melvin',
    maintainer_email='melvin@ryugu.sim',
    description='Team Jalpari Ryugu Asteroid Swarm Simulation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'attitude_controller = ryugu_sim.attitude_controller:main',
            'swarm_manager = ryugu_sim.swarm_manager:main',
            'spawner = ryugu_sim.spawner:main',
            'hopper_locomotion = ryugu_sim.hopper_locomotion:main',
            'landing_controller = ryugu_sim.landing_controller:main',
        ],
    },
)
