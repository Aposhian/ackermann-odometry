from setuptools import setup
import os
from glob import glob

package_name = 'ackermann_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adam Aposhian',
    maintainer_email='aposhian.dev@gmail.com',
    description='Ackermann Control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ackermann_teleop_joy = ackermann_controller.ackermann_teleop_joy:main'
        ],
    },
)
