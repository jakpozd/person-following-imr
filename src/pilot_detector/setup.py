from setuptools import find_packages, setup

package_name = 'pilot_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name],),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pilot_follower_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    # maintainer_email='jetson@todo.todo',
    description='Detection and following of human pilot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_node = pilot_detector.detector_node:main',
            'follower_node = pilot_detector.follower_node:main'
        ],
    },
)
