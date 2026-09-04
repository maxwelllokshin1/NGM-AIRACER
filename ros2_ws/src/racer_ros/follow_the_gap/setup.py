from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'follow_the_gap'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'dashboard'),
            glob(os.path.join('dashboard', '*.html')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='maxwell.lokshin@2zick.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reactive_node = follow_the_gap.reactive_node:main',
            'car_info_monitor = follow_the_gap.car_info_monitor:main',
            'car_dashboard = follow_the_gap.car_dashboard:main'
        ],
    },
)
