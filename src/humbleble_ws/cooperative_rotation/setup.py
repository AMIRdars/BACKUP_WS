from glob import glob
from setuptools import find_packages, setup

package_name = 'cooperative_rotation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dars5070',
    maintainer_email='dars5070@todo.todo',
    description='Specified-axis planner for cooperative payload rotation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rotation_controller = '
            'cooperative_rotation.rotation_controller:main',
            'rotation_data_recorder = '
            'cooperative_rotation.rotation_data_recorder:main',
        ],
    },
)
