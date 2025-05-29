import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'basic_mobile_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'models'),
            glob(os.path.join('models', '**', '*.*'), recursive=True)),
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '**', '*.world'), recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ibz',
    maintainer_email='issakaibrahimrayamah@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)


