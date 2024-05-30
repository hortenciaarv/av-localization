from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='a01639250@tec.mx',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinematic_model = localization.kinematic_model:main',
            'localization = localization.localization:main',
            'coordinate_transform = localization.coordinate_transform:main',
            'controller = localization.controller:main',
            'joint_state = localization.joint_states:main',
            'to_goal = localization.to_goal:main',
            'to_goal_bug_2 = localization.to_goal_bug2:main',
            'test_lidar = localization.test_lidar:main',
            'bug0 = localization.bug0:main',
            'aruco_controller = localization.aruco_controller:main',
            'transform_listener = localization.transform_listener:main',
            'to_goal_aruco = localization.to_goal_aruco:main',

        ],
    },
)
