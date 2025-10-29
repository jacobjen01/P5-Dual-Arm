from setuptools import find_packages, setup

package_name = 'p5_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/p5_controller/launch', ['launch/robot_commands_listener.launch.py'])
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
            'change_pre_config_poses = p5_controller.change_pre_config_poses:main',
            'move_to_pre_config_poses = p5_controller.move_to_pre_config_poses:main',
            'template = p5_controller.template:main',
            'relative_mover = p5_controller.relative_mover_node:main',
            'home = p5_controller.home:main',
            'test_move = p5_controller.test_of_rm:main',
            'admittance_node = p5_controller.admittance_node:main'
        ],
    },
)
