import os
from glob import glob

from setuptools import setup

package_name = 'my_nav_pkg'

setup(
    name=package_name,
    version='0.0.1',
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
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Global-EKF spline waypoint navigation with optional local obstacle avoidance',
    license='MIT',
    entry_points={
        'console_scripts': [
            'waypoint_follower_node = my_nav_pkg.waypoint_follower_node:main',
            'waypoint_recorder_node = my_nav_pkg.waypoint_recorder_node:main',
            'main_controller_node = my_nav_pkg.main_controller_node:main',
            'local_avoider_node = my_nav_pkg.local_avoider_node:main',
            'parking_controller_node = my_nav_pkg.parking_controller_node:main',
            'zed_f9p_usb_rtcm = my_nav_pkg.zed_f9p_usb_rtcm:main',
            'imu_enu_adapter_node = my_nav_pkg.imu_enu_adapter_node:main',
            'heading_selector_node = my_nav_pkg.heading_selector_node:main',
            'wheel_twist_adapter_node = my_nav_pkg.wheel_twist_adapter_node:main',
            'gnss_enu_odometry_node = my_nav_pkg.gnss_enu_odometry_node:main',
        ],
    },
)
