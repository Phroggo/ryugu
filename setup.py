from setuptools import setup
import os
from glob import glob

package_name = 'ryugu_sim'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.py')),
    ],
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
