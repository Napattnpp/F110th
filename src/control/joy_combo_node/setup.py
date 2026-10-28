import os
from glob import glob
from setuptools import setup

package_name = 'joy_combo_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Napat',
    maintainer_email='napat@example.com',
    description='Dual-function gamepad modifier and combo dispatcher for F1TENTH racing operations',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_combo_node = joy_combo_node.joy_combo_node:main',
        ],
    },
)

