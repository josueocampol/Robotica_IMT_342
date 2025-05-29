from setuptools import find_packages, setup

package_name = 'mi_proyecto_de_ros2'

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
    maintainer='josue',
    maintainer_email='josue@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mi_primer_nodo = mi_proyecto_de_ros2.mi_primer_nodo:main',
            'mi_nodo_teleop = mi_proyecto_de_ros2.mi_nodo_teleop:main',
        ],
    },
)
