"""Read measured ungrasped speed evidence; never synthesize a passing case."""
import json
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import yaml


def combine_evidence(folder, generated, original):
    folder, generated = Path(folder), Path(generated)
    result = json.loads((folder/'result.json').read_text())
    cases = json.loads((folder/'motion.json').read_text())
    if (not result.get('passed') or not cases or not all(c.get('passed') for c in cases)
            or result.get('scope') != 'simulation-only ungrasped base motion; not loaded transport'):
        raise ValueError('complete passing CP-10 base evidence required')
    combined = [v for v in json.loads(Path(original).read_text()) if v['test'] != 'base_x']
    for robot in ('leader','follower'):
        model = result[robot+'_model']
        config = yaml.safe_load((generated/(robot+'_base_controller.yaml')).read_text())
        params = config['/**/mecanum_drive_controller']['ros__parameters']
        if model['wheel_radius_m'] != params['kinematics']['wheels_radius']:
            raise ValueError('wheel radius differs from measured model')
        root = ET.parse(generated/(robot+'.urdf')).getroot()
        for name, measured in model['wheel_limits_rad_s'].items():
            limit = float(root.find(f"joint[@name='{name}']/limit").get('velocity'))
            if limit != measured:
                raise ValueError('wheel limit differs from measured model')
        positive = [v for v in cases if v['robot']==robot and v['test']=='base_x' and v['speed'] > 0]
        if not positive:
            raise ValueError('missing positive base speed proof')
        best = max(positive,key=lambda c:c['speed'])
        negatives = [v for v in cases if v['robot']==robot and v['test']=='base_x' and v['speed']==-best['speed']]
        if not negatives or best['speed'] < 1.05:
            raise ValueError('both signs through 1.05 m/s required for tracking reserve')
        for case in (best, negatives[0]):
            measured = case['steady_gt_speed_m_s']
            if not math.isfinite(measured) or abs(measured/case['speed']-1) >= .1:
                raise ValueError('measured speed does not confirm command')
        combined.append({**best, 'evidence_directory':str(folder.resolve()),
                         'negative_speed_proof':negatives[0]})
    return combined
