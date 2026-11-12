"""Clone the certified native scene, changing only evaluation publication rates."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


def display_world(data):
    root=ET.fromstring(data)
    plugin=root.find("world/plugin[@name='ignition::gazebo::systems::SceneBroadcaster']")
    if plugin is None or list(plugin):
        raise ValueError('expected the original unconfigured SceneBroadcaster')
    # Installed Fortress 6.16 implements both keys; these publish GUI state,
    # never step the physics engine or alter sensor/actuator update rates.
    for key in ('state_hertz','dynamic_pose_hertz'):
        ET.SubElement(plugin,key).text='20'
    return ET.tostring(root,encoding='utf-8',xml_declaration=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise ValueError('preserve existing generated scene')
    shutil.copytree(a.source,a.output)
    world=a.output/'world.sdf';original=world.read_bytes();world.write_bytes(display_world(original))
    # Compare the complete XML tree after removing exactly the two display
    # settings. Every physical property, plugin and original text is retained.
    stripped=ET.fromstring(world.read_bytes())
    plugin=stripped.find("world/plugin[@name='ignition::gazebo::systems::SceneBroadcaster']")
    for child in list(plugin):plugin.remove(child)
    assert ET.tostring(stripped)==ET.tostring(ET.fromstring(original))
    sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
    unchanged={str(f.relative_to(a.source)):sha(f)==sha(a.output/f.relative_to(a.source))
        for f in a.source.rglob('*') if f.is_file() and f.name!='world.sdf'}
    assert all(unchanged.values())
    (a.output/'display_only_equivalence.json').write_text(json.dumps({
        'passed':True,'source':str(a.source.resolve()),'state_hertz':20,'dynamic_pose_hertz':20,
        'all_other_world_xml_equal':True,'other_files_equal':unchanged,
        'source_world_sha256':hashlib.sha256(original).hexdigest(),'world_sha256':sha(world),
        'api_source':'https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/scene_broadcaster/SceneBroadcaster.cc'},indent=2))


if __name__=='__main__':main()
