"""Implement the collision-count experiment after the baseline finishes."""
from pathlib import Path
import xml.etree.ElementTree as E,re
root=Path('src/humbleble_ws')
p=root/'amir740_ros/amir_description/urdf/amir_for_rover.xacro'
s=p.read_bytes().decode()
if '<xacro:arg name="finger_collision_boxes"' in s:
    raise SystemExit('Collision experiment is already implemented; refusing to apply twice.')
ns={'x':'http://www.ros.org/wiki/xacro'}; tree=E.fromstring(s)
# Preserve nine visual boxes and all inertial/joint definitions.
s=s.replace('<!-- Emit one identical visual/collision pair.', '<xacro:arg name="finger_collision_boxes" default="9"/>\n<xacro:property name="finger_collision_boxes" value="$(arg finger_collision_boxes)"/>\n\n<!-- Emit one identical visual/collision pair.',1)
start=s.index('<xacro:macro name="finger_box"');end=s.index('</xacro:macro>',start)
macro=s[start:end];macro=macro.replace('  <collision>','  <xacro:if value="${int(finger_collision_boxes) == 9}">\n  <collision>',1).replace('  </collision>','  </collision>\n  </xacro:if>',1)
s=s[:start]+macro+s[end:]
extra='''\n<xacro:macro name="finger_collision_box" params="xyz size">
  <collision><origin xyz="${xyz}" rpy="0 0 0"/>
    <geometry><box size="${size}"/></geometry>
  </collision>
</xacro:macro>\n'''
pos=s.index('</xacro:macro>')+len('</xacro:macro>');s=s[:pos]+extra+s[pos:]
for side in ['left','right']:
 link=tree.find(f"link[@name='finger_{side}_1']")
 boxes=[]
 for box in link.findall('x:finger_box',ns):
  c=list(map(float,box.attrib['xyz'].split()));d=list(map(float,box.attrib['size'].split()));boxes.append((c,d))
 def box_xml(indices):
  low=[min(boxes[i][0][j]-boxes[i][1][j]/2 for i in indices) for j in range(3)]
  high=[max(boxes[i][0][j]+boxes[i][1][j]/2 for i in indices) for j in range(3)]
  xyz=' '.join(f'{(lo+hi)/2:.6f}' for lo,hi in zip(low,high));size=' '.join(f'{hi-lo:.6f}' for lo,hi in zip(low,high))
  return f'    <xacro:finger_collision_box xyz="{xyz}" size="{size}"/>\n'
 xml='  <!-- Collision experiment: original contact pad retained exactly in 3-box mode. -->\n'
 xml+='  <xacro:if value="${int(finger_collision_boxes) == 3}">\n'+box_xml([0])+box_xml(list(range(1,6)))+box_xml(list(range(6,9)))+'  </xacro:if>\n'
 xml+='  <xacro:if value="${int(finger_collision_boxes) == 1}">\n'+box_xml(list(range(9)))+'  </xacro:if>\n'
 start=s.index(f'<link name="finger_{side}_1">');end=s.index('</link>',start);s=s[:end]+xml+s[end:]
p.write_bytes(s.encode())
p=root/'amir740_ros/amir_description/urdf/amir_mecanum3_sim.xacro';s=p.read_text()
for side in ['left','right']:
 for i in range(1,9):
  tag=f'    <xacro:finger_segment_contact side="{side}" suffix="_{i}" collision_suffix="_{i}"/>'
  s=s.replace(tag,f'    <xacro:if value="${{int(finger_collision_boxes) > {i}}}">\n{tag}\n    </xacro:if>')
p.write_text(s)
p=root/'amir740_ros/amir_gazebo/launch/robot_bringup.launch.py';s=p.read_text()
s=s.replace('    # prefix 文字列:', '''    finger_collision_boxes = int(LaunchConfiguration("finger_collision_boxes").perform(context))
    if finger_collision_boxes not in (1, 3, 9):
        raise ValueError("finger_collision_boxes must be 1, 3, or 9")

    # prefix 文字列:''',1)
s=s.replace('"namespace": ns,\n            "enable_d435"','"namespace": ns,\n            "finger_collision_boxes": str(finger_collision_boxes),\n            "enable_d435"',1)
s=s.replace('for index in range(9):','for index in range(finger_collision_boxes):')
s=s.replace('        DeclareLaunchArgument("namespace", default_value=""),','        DeclareLaunchArgument("finger_collision_boxes", default_value="9"),\n        DeclareLaunchArgument("namespace", default_value=""),',1)
p.write_text(s)
p=root/'cooperative_transport_bringup/launch/friction_simulation.launch.py';s=p.read_text()
s=s.replace("'namespace': namespace,", "'namespace': namespace,\n                'finger_collision_boxes': LaunchConfiguration('finger_collision_boxes'),",1)
s=s.replace("        DeclareLaunchArgument('headless', default_value='false'),","        DeclareLaunchArgument('finger_collision_boxes', default_value='9'),\n        DeclareLaunchArgument('headless', default_value='false'),",1)
p.write_text(s)
p=root/'cooperative_transport_bringup/launch/integrated_transport_simulation.launch.py';s=p.read_text()
s=s.replace("'headless': LaunchConfiguration('headless'),", "'headless': LaunchConfiguration('headless'),\n            'finger_collision_boxes': LaunchConfiguration('finger_collision_boxes'),",1)
s=s.replace("        DeclareLaunchArgument('headless', default_value='false'),","        DeclareLaunchArgument('finger_collision_boxes', default_value='9'),\n        DeclareLaunchArgument('headless', default_value='false'),",1)
p.write_text(s)
