from setuptools import setup, find_packages

package_name = 'ardupilot_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ardupilot_gui.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'PyQt5',
        'numpy',
        'geometry_msgs',
        'sensor_msgs',
        'nav_msgs',
        'std_msgs',
        'ardupilot_msgs',
    ],
    zip_safe=True,
    maintainer='Stephen Dade',
    maintainer_email='stephen_dade@hotmail.com',
    description='ArduPilot ROS2 GUI for monitoring and control',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ardupilot_gui = ardupilot_gui.main:main',
        ],
    },
)
