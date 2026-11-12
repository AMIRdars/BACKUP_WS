#!/usr/bin/env python3
"""Offline conservative base/base clearance from actual URDF/STL and saved GT.

Includes chassis and every ancestor link of the configured wheels. It does not
certify arm/payload/environment clearance or replace a runtime collision monitor.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

from amir_compliance_control.wrench import origin


def mesh_vertices(mesh):
    uri = mesh.get('filename')
    if not uri.startswith('package://'):
        raise ValueError('inspect unsupported mesh URI: '+uri)
    package, relative = uri[len('package://'):].split('/', 1)
    file = Path(get_package_share_directory(package))/relative
    data = file.read_bytes()
    if len(data) >= 84 and len(data) == 84+50*struct.unpack_from('<I', data, 80)[0]:
        facets = np.frombuffer(data, dtype=np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')]), offset=84)
        vertices = facets['vertices'].reshape(-1, 3).astype(float)
    else:
        vertices = np.array([[float(v) for v in line.split()[1:]]
                             for line in data.decode().splitlines() if line.strip().startswith('vertex ')])
    if vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) == 0:
        raise ValueError('cannot inspect STL vertices')
    scale = np.array(mesh.get('scale', '1 1 1').split(), float)
    return vertices*scale, {'path': str(file.resolve()), 'sha256': hashlib.sha256(data).hexdigest(), 'scale': scale.tolist()}


def base_bound(folder, robot):
    root = ET.parse(folder/(robot+'.urdf')).getroot()
    parents = {j.find('child').get('link'): j for j in root.findall('joint')}
    config = yaml.safe_load((folder/(robot+'_base_controller.yaml')).read_text())
    params = config['/**/mecanum_drive_controller']['ros__parameters']
    joints = [params[k] for k in params if k.endswith('_wheel_command_joint_name')]
    links = {'base_footprint'}
    for name in joints:
        link = root.find(f"joint[@name='{name}']/child").get('link')
        while link != 'base_footprint':
            links.add(link)
            link = parents[link].find('parent').get('link')

    def transform(link):
        return np.eye(4) if link == 'base_footprint' else transform(parents[link].find('parent').get('link'))@origin(parents[link].find('origin'))

    bounds, meshes = {}, []
    for link in sorted(links):
        for i, collision in enumerate(root.findall(f"link[@name='{link}']/collision")):
            t = transform(link)@origin(collision.find('origin'))
            geometry = collision.find('geometry')
            mesh, cylinder, box = geometry.find('mesh'), geometry.find('cylinder'), geometry.find('box')
            if mesh is not None:
                v, metadata = mesh_vertices(mesh)
                points = v@t[:3, :3].T+t[:3, 3]
                radius = float(np.max(np.linalg.norm(points, axis=1)))
                meshes.append(metadata)
            elif cylinder is not None:
                radius = float(np.linalg.norm(t[:3, 3])+np.hypot(float(cylinder.get('radius')), float(cylinder.get('length'))/2.))
            elif box is not None:
                radius = float(np.linalg.norm(t[:3, 3])+np.linalg.norm(np.array(box.get('size').split(), float)/2.))
            else:
                raise ValueError('unsupported chassis collision geometry')
            bounds[link+'/'+str(i)] = radius
    return {'bounding_sphere_radius_m': max(bounds.values()), 'link_collision_bounds_m': bounds, 'meshes': meshes}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--audit-directory', type=Path, required=True)
    a = p.parse_args()
    bounds = {r: base_bound(a.audit_directory/'generated', r) for r in ('leader', 'follower')}
    total = sum(b['bounding_sphere_radius_m'] for b in bounds.values())
    trials = {}
    for path in sorted(a.audit_directory.glob('*/samples.jsonl')):
        rows = [json.loads(line) for line in path.open()]
        distances = [np.linalg.norm(np.array([row['leader_GT_'+k]-row['follower_GT_'+k] for k in 'xyz']))
                     for row in rows if row['phase'] in ('rotation', 'hold') and 'leader_GT_x' in row and 'follower_GT_x' in row]
        if distances:
            minimum = float(min(distances))
            trials[path.parent.name] = {'minimum_base_origin_distance_m': minimum,
                'conservative_minimum_base_surface_gap_m': minimum-total, 'separated': minimum > total}
    report = {'scope': __doc__, 'base_geometry': bounds, 'trials': trials,
              'all_recorded_motion_bases_separated': bool(trials) and all(v['separated'] for v in trials.values())}
    (a.audit_directory/'base_clearance.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({'all_recorded_motion_bases_separated': report['all_recorded_motion_bases_separated'], 'trials': trials}, indent=2))
    return 0 if report['all_recorded_motion_bases_separated'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
