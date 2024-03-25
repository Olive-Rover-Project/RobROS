from setuptools import find_packages, setup

package_name = 'gps_converter'

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
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_converter_node = gps_converter.gps_converter_node:main',
            'increment_goal_node = gps_converter.increment_goal_node:main',
            'initial_position_averager = gps_converter.initial_position_averager:main',
            'rover_initialiser = gps_converter.rover_initialiser:main',
        ],
    },
)
