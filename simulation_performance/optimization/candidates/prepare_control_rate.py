from pathlib import Path
root=Path('src/humbleble_ws')
p=root/'cooperative_transport_control/cooperative_transport_control/slip_monitor.py';s=p.read_text().replace("        self.declare_parameter('world_frame', 'world')", "        self.declare_parameter('world_frame', 'world')\n        self.declare_parameter('monitor_rate', 50.0)")
s=s.replace('        self.create_timer(0.02, self._tick)', "        rate = float(self.get_parameter('monitor_rate').value)\n        if rate <= 0.0:\n            raise ValueError('monitor_rate must be positive')\n        self.create_timer(1.0 / rate, self._tick)")
p.write_text(s)
p=root/'cooperative_transport_bringup/launch/friction_simulation.launch.py';s=p.read_text().replace("'horizontal_slip_limit': horizontal_slip_limit,", "'monitor_rate': float(LaunchConfiguration('slip_monitor_rate').perform(context)),\n            'horizontal_slip_limit': horizontal_slip_limit,")
s=s.replace("        DeclareLaunchArgument('headless'", "        DeclareLaunchArgument('slip_monitor_rate', default_value='50.0'),\n        DeclareLaunchArgument('headless'",1);p.write_text(s)
p=root/'cooperative_transport_bringup/launch/integrated_transport_simulation.launch.py';s=p.read_text().replace("            'headless': LaunchConfiguration('headless'),", "            'headless': LaunchConfiguration('headless'),\n            'slip_monitor_rate': LaunchConfiguration('slip_monitor_rate'),")
s=s.replace("        DeclareLaunchArgument('headless'", "        DeclareLaunchArgument('slip_monitor_rate', default_value='50.0'),\n        DeclareLaunchArgument('headless'",1);p.write_text(s)
