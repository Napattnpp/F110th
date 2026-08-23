import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'global_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        'global_planner',
        'global_planner.global_racetrajectory_optimization',
        'global_planner.global_racetrajectory_optimization.global_racetrajectory_optimization',
        'global_planner.global_racetrajectory_optimization.global_racetrajectory_optimization.helper_funcs_glob',
        'global_planner.global_racetrajectory_optimization.global_racetrajectory_optimization.helper_funcs_glob.src',
        'global_planner.global_racetrajectory_optimization.global_racetrajectory_optimization.opt_mintime_traj',
        'global_planner.global_racetrajectory_optimization.global_racetrajectory_optimization.opt_mintime_traj.src',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ForzaETH',
    maintainer_email='nicolas.baumann@pbl.ee.ethz.ch',
    description='The global planner package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_planner = global_planner.global_planner_node:main',
            'global_trajectory_publisher = global_planner.global_trajectory_publisher:main',
            'manual_waypoint_logger = global_planner.manual_waypoint_logger:main'
        ],
    },
)
