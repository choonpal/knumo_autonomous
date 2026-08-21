import os
from glob import glob

from setuptools import setup


package_name = 'autodrive_pkg'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # YOLO 신호등 모델. vision.py 가 model_path 로 이 파일을 읽는다.
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bansong',
    maintainer_email='bansong@todo.todo',
    description='Regulation-aware mission stop supervisor and vision',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_control_node = autodrive_pkg.control:main',
            'vision_node = autodrive_pkg.vision:main',
        ],
    },
)
