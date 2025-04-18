from setuptools import setup

package_name = 'wall_filter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brady',
    maintainer_email='bradyward@u.boisestate.edu',
    description='Basic bounding box point filter for rplidar and unitree lidar devices',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_filter_node = wall_filter.wall_filter_node:main',
        ],
    },
)

