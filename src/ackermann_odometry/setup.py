from setuptools import setup

package_name = 'ackermann_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adam Aposhian',
    maintainer_email='aposhian.dev@gmail.com',
    description='ROS Node to publish Odometry message for ackermann vehicle based on wheel speed and steering angle',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_publisher = ackermann_odometry.odom_publisher:main'
        ],
    },
)
