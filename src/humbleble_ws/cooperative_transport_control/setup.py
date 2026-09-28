from glob import glob
from setuptools import find_packages, setup

package_name = 'cooperative_transport_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dars5070',
    maintainer_email='dars5070@todo.todo',
    description='Cooperative transport control nodes for two AMIR robots.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coordinator = cooperative_transport_control.coordinator:main',
            'dual_base_approach = '
            'cooperative_transport_control.dual_base_approach:main',
            'attach_manager = cooperative_transport_control.attach_manager:main',
            'friction_grasp_manager = cooperative_transport_control.friction_grasp_manager:main',
            'grasp_support_remover = cooperative_transport_control.grasp_support_remover:main',
            'trajectory_evaluator = cooperative_transport_control.trajectory_evaluator:main',
            'safety_monitor = cooperative_transport_control.safety_monitor:main',
            'slip_monitor = cooperative_transport_control.slip_monitor:main',
            'wrench_monitor = cooperative_transport_control.wrench_monitor:main',
            'single_robot_lateral_translation = '
            'cooperative_transport_control.single_robot_lateral_translation:main',
            'cooperative_lateral_goal = '
            'cooperative_transport_control.cooperative_lateral_goal:main',
            'pivot_rotation_controller = '
            'cooperative_transport_control.pivot_rotation_controller:main',
        ],
    },
)
