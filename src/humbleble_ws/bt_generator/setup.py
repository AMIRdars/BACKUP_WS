from setuptools import setup
import os
from glob import glob

package_name = 'bt_generator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
        (os.path.join('share', package_name), glob('bt_xml/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kawase',
    maintainer_email='kawase.haruyoshi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bt_generator_service = bt_generator.bt_generator_service:main',
            'bt_commander = bt_generator.bt_commander:main',
            'bt_send_xml = bt_generator.bt_send_xml:main',
            'receive_send_xml = bt_generator.receive_send_xml:main',
            'bt_generator_service_bebop = bt_generator.bt_generator_service_bebop:main',
            'bt_generator_bebop_base = bt_generator.bt_generator_bebop_base:main'
        ],
    },
)
