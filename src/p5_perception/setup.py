from setuptools import find_packages, setup

package_name = 'p5_perception'

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
    maintainer='jacob',
    maintainer_email='jacobjen01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'future_tag_estimator = p5_perception.future_tag_estimator:main',
            'future_tag_estimator_quick = p5_perception.future_tag_estimator_quick:main',
            'future_tag_estimator_with_timing = p5_perception.future_tag_estimator_with_timing:main',
            'future_tag_estimator_quick_with_timing = p5_perception.future_tag_estimator_quick_with_timing:main',
        ],
    },
)
