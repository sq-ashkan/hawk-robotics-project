import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'line_follower_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world') + glob('worlds/*.sdf')),
        # Include model files
        (os.path.join('share', package_name, 'models', 'white_line'), glob('models/white_line/*')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'numpy',
        'pyyaml',
    ],
    zip_safe=True,
    maintainer='roammer',
    maintainer_email='ashkan.sadri.ghamshi@gmail.com',
    description='TurtleBot3 Line Follower - Camera calibration, Birds-Eye View, and Line Following',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'line_follower_node = line_follower_pkg.line_follower_node:main',
            'real_robot_line_follower = line_follower_pkg.real_robot.real_robot_line_follower_node:main',
            'calibrate_camera = line_follower_pkg.calibrate_camera:main',
            'calibrate_homography = line_follower_pkg.calibrate_homography:main',
        ],
    },
)
