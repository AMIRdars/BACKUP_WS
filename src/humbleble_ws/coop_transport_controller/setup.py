from setuptools import find_packages, setup

package_name = 'coop_transport_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/controller.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dars5070',
    maintainer_email='dars5070@todo.todo',
    description='ROS 2 Action server for cooperative transport motions.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'cooperative_motion_server = coop_transport_controller.cooperative_motion_server:main',
    ]},
)
