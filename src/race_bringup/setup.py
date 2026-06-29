import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'race_bringup'


def package_files(destination, source):
    paths = []
    for path in glob(os.path.join(source, '**', '*'), recursive=True):
        if os.path.isfile(path):
            paths.append((os.path.join('share', package_name, destination, os.path.relpath(os.path.dirname(path), source)), [path]))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ]
    + package_files('config', 'config')
    + package_files('maps', 'maps'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='napat',
    maintainer_email='nnapat156@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'global_parameter_node = race_bringup.global_parameter_node:main',
        ],
    },
)
