from setuptools import setup
import os
from glob import glob

package_name = 'm0609_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name, ['bin/server']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parksangwook',
    maintainer_email='tkddnr1022@dankook.ac.kr',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sub_joint = m0609_controller.sub_joint:main',
            
        ],
    },
)
