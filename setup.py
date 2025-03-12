from setuptools import find_packages, setup

package_name = 'node_testing_py'

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
    maintainer='brady',
    maintainer_email='me@me.com',
    description='Basic publisher and subscriber nodes using rlcpy',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = node_testing_py.publisher:main',
             'listener = node_testing_py.subscriber:main',
        ],
    },
)
