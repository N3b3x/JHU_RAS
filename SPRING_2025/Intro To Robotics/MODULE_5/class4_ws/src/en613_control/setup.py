import os 
from glob import glob
from setuptools import find_packages, setup

package_name = 'en613_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name,'launch'), glob('launch/*.rviz')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='n3b3x',
    maintainer_email='nebysma@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diffdrive_sim = en613_control.diffdrive_sim:main',
            'diffdrive_pid = en613_control.diffdrive_pid:main',
        ],
    },
)
