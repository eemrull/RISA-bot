from setuptools import setup
import os
from glob import glob

package_name = 'risabot_automode'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # ADD THIS LINE BELOW:
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@todo.todo',
    description='RisaBot Auto Mode Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # The Entry Point: Name = Package.File:Function
            'auto_driver = risabot_automode.auto_driver:main',
            'line_follower_camera = risabot_automode.line_follower_camera:main',
        ],
    },
)
