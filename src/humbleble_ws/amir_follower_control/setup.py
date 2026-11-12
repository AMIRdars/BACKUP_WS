from glob import glob
from setuptools import setup

setup(name='amir_follower_control', version='0.1.0',
      packages=['amir_follower_control'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/amir_follower_control']),
                  ('share/amir_follower_control', ['package.xml', 'README.md']),
                  ('share/amir_follower_control/config', glob('config/*.yaml')),
                  ('share/amir_follower_control/launch', glob('launch/*.launch.py')),
                  ('share/amir_follower_control/experiments', glob('experiments/*.py'))],
      install_requires=['setuptools', 'numpy'], zip_safe=True,
      maintainer='dars5070', maintainer_email='dars5070@todo.todo', license='Apache-2.0',
      entry_points={'console_scripts': [
          'follower_controller = amir_follower_control.node:main',
          'home_once = amir_follower_control.startup:home_main',
          'approach_once = amir_follower_control.startup:approach_main',
          'gentle_attach = amir_follower_control.gentle_grasp:main',
          'gentle_friction_grasp = amir_follower_control.gentle_grasp:friction_main',
          'managed_attach = amir_follower_control.gentle_grasp:managed_main',
          'wait_grasp_ready = amir_follower_control.startup:wait_grasp_main',
          'wait_grasp_start = amir_follower_control.startup:wait_grasp_start_main',
          'prepare_payload = amir_follower_control.prepare_payload:main',
          'local_evidence = amir_follower_control.evidence:main']})
