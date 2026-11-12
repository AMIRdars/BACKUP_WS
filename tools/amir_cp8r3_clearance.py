"""Evaluation-only base separation guard from actual URDF/STL spheres.

The 10% sum-radius reserve is an explicit simulation guard, not a physical
braking-distance certification. No pose or centre is sent to either robot.
"""
import numpy as np
import xml.etree.ElementTree as ET
from pathlib import Path
from amir_cp8r1_base_clearance import base_bound, mesh_vertices
from amir_compliance_control.wrench import origin


def all_wheel_angles_bound(folder,robot):
    bound=base_bound(folder,robot)
    root=ET.parse(folder/(robot+'.urdf')).getroot()
    parents={j.find('child').get('link'):j for j in root.findall('joint')}
    def chain_length(link):
        if link=='base_footprint':return 0.
        joint=parents[link]
        return chain_length(joint.find('parent').get('link'))+float(np.linalg.norm(origin(joint.find('origin'))[:3,3]))
    upper={}
    for key in bound['link_collision_bounds_m']:
        link,index=key.rsplit('/',1)
        collision=root.findall("link[@name='"+link+"']/collision")[int(index)]
        geometry=collision.find('geometry');mesh=geometry.find('mesh');box=geometry.find('box');cylinder=geometry.find('cylinder')
        if mesh is not None:
            vertices,_=mesh_vertices(mesh);radius=float(np.max(np.linalg.norm(vertices,axis=1)))
        elif box is not None:radius=float(np.linalg.norm(np.array(box.get('size').split(),float)/2.))
        elif cylinder is not None:radius=float(np.hypot(float(cylinder.get('radius')),float(cylinder.get('length'))/2.))
        else:raise ValueError('unsupported base collision shape')
        # Triangle inequality bounds every ancestor rotation, including all
        # wheel phases; a q=0 STL orientation alone is insufficient for that.
        upper[key]=chain_length(link)+float(np.linalg.norm(origin(collision.find('origin'))[:3,3]))+radius
    bound['all_joint_angles_collision_bounds_m']=upper
    bound['bounding_sphere_radius_m']=max(max(upper.values()),bound['bounding_sphere_radius_m'])
    return bound


class BaseClearance:
    def __init__(self, folder):
        self.bounds = {r:all_wheel_angles_bound(folder,r) for r in ('leader','follower')}
        self.total = sum(v['bounding_sphere_radius_m'] for v in self.bounds.values())
        self.reserve = .1*self.total

    def gap(self, leader, follower):
        return float(np.linalg.norm(leader[:3,3]-follower[:3,3])-self.total)


def main():
    """Check every saved motion pose with the all-wheel-phase bound."""
    import argparse
    import json
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--audit-directory', type=Path, required=True)
    a = p.parse_args()
    geometry, trials = {}, {}
    for result_file in sorted(a.audit_directory.glob('*/result.json')):
        folder = result_file.parent
        samples = folder/'samples.jsonl'
        if not samples.exists():
            continue
        result = json.loads(result_file.read_text())
        name = 'native_generated' if result.get('condition', '').startswith('native_') else 'generated'
        if name not in geometry:
            geometry[name] = BaseClearance(a.audit_directory/name)
        guard = geometry[name]
        minimum, count = None, 0
        for line in samples.open():
            row = json.loads(line)
            if row['phase'] not in ('rotation', 'hold') or not all(
                    robot+'_GT_'+axis in row for robot in ('leader', 'follower') for axis in 'xyz'):
                continue
            distance = float(np.linalg.norm([row['leader_GT_'+axis]-row['follower_GT_'+axis] for axis in 'xyz']))
            minimum = distance if minimum is None else min(minimum, distance)
            count += 1
        if count:
            trials[folder.name] = dict(motion_samples=count, geometry=name,
                minimum_base_origin_distance_m=minimum,
                conservative_minimum_surface_gap_m=minimum-guard.total,
                reserve_m=guard.reserve, reserve_preserved=minimum-guard.total>guard.reserve)
    report = dict(scope=__doc__, all_recorded_motion_reserves_preserved=bool(trials) and all(
        r['reserve_preserved'] for r in trials.values()), trials=trials,
        geometry={name:dict(bounds=g.bounds,reserve_m=g.reserve) for name,g in geometry.items()})
    (a.audit_directory/'base_clearance.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='geometry'},indent=2))
    return 0 if report['all_recorded_motion_reserves_preserved'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
