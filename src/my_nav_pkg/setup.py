from setuptools import setup

package_name = 'my_nav_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/nav_system_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Waypoint navigation system',
    license='MIT',
    entry_points={
        'console_scripts': [
            'waypoint_follower_node   = my_nav_pkg.waypoint_follower_node:main',
            'waypoint_recorder_node   = my_nav_pkg.waypoint_recorder_node:main',
            'main_controller_node     = my_nav_pkg.main_controller_node:main',
            'test_pub_node            = my_nav_pkg.test_pub_node:main',
            'obstacle_avoider_node    = my_nav_pkg.obstacle_avoider_node:main',
        ],
    },
)

