from setuptools import find_packages, setup

package_name = 'p5_safety'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/p5_safety/launch', ['launch/launch_safety.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            '_error_handling = p5_safety._error_handling:main',
            'collision_detector_node = p5_safety.collision_detector_node:main',
            'robot_protective_stop_node = p5_safety.robot_protective_stop_node:main'
        ],
    },
)
