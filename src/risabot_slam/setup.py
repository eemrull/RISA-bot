from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'risabot_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eemrull',
    maintainer_email='eemrull@localhost',
    description='Isolated slam_toolbox mapping test stack for RISA-bot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_tf_publisher = risabot_slam.odom_tf_publisher:main',
            'scan_restamper = risabot_slam.scan_restamper:main',
        ],
    },
)
