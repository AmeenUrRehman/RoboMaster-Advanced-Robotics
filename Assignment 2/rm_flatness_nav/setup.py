from glob import glob

from setuptools import find_packages, setup

package_name = 'rm_flatness_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Differential-flatness AprilTag pursuit with motor-voltage logging for the RoboMaster EP',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flatness_nav = rm_flatness_nav.nav_node:main',
            'flatness_sim = rm_flatness_nav.sim:main',
            'camera_pose = rm_flatness_nav.camera_pose:main',
        ],
    },
)
