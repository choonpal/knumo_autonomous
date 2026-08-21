import os
from glob import glob

from setuptools import setup


package_name = 'competition_bringup'


setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bansong',
    maintainer_email='bansong@todo.todo',
    description='KNU Global EKF and Pure Pursuit waypoint drive bringup',
    license='MIT',
    tests_require=['pytest'],
)
