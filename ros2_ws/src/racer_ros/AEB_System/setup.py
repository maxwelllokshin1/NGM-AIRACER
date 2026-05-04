from setuptools import setup
import os # Added
from glob import glob # Added

package_name = 'AEB_System'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student Name',
    maintainer_email='student@example.com',
    description='AEB safety node using iTTC',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #considerded as maybe as executables? 
            'safety_node = AEB_System.safety_node:main',
        ],
    },
)