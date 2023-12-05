from setuptools import setup
import os
from glob import glob


package_name = 'anyword'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),      

    ],
    install_requires=[
        'setuptools',
        'rosdep',
        'ament_python',
        'std_msgs',
        'sensor_msgs',
        'std_srvs',
        'rclpy',
        'geometry_msgs'


        # Add any additional Python dependencies here
    ],
    zip_safe=True,
    author='Owen Bates',
    author_email='okb6@cornell.edu',
    maintainer='Owen Bates',
    maintainer_email='okb6@cornell.edu',
    description='anyword',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'anyword = anyword.anyword:main'
        ],
    },
    include_package_data=True
)
