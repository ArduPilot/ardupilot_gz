from setuptools import find_packages, setup

package_name = 'ardupilot_gz_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rhys Mainwaring',
    maintainer_email='rhys.mainwaring@me.com',
    description='Utilities for running ArduPilot with Gazebo',
    license='GPLv3',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'send_bool = ardupilot_gz_utils.send_bool:main',
            'wait_for_bool = ardupilot_gz_utils.wait_for_bool:main'
        ],
    },
)
