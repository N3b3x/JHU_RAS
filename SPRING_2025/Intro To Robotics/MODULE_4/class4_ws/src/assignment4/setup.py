from setuptools import find_packages, setup

package_name = 'assignment4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'mecanum_sim = assignment4.mecanum_sim:main',
            'teleop_twist_keyboard = assignment4.teleop_twist_keyboard:main'
        ],
    },
)
