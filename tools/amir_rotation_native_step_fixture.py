"""Step-halving diagnostic on a fresh native fixture; no transport authority."""
import copy
import hashlib
import json
import math
import xml.etree.ElementTree as ET


def refine(fixture, divisor):
    if type(divisor) is not int or divisor not in (2,4):
        raise ValueError('only explicit step-halving divisors 2 or 4 allowed')
    world=fixture/'world.sdf';original=world.read_bytes()
    root=ET.fromstring(original);physics=root.findall('world/physics')
    if len(physics)!=1 or len(physics[0].findall('max_step_size'))!=1:
        raise ValueError('one explicit native physics step required')
    step=physics[0].find('max_step_size');previous=step.text;baseline=float(previous)
    if not math.isfinite(baseline) or baseline<=0.:
        raise ValueError('finite positive baseline step required')
    step.text=repr(baseline/divisor)
    changed=ET.tostring(root,encoding='utf-8',xml_declaration=True)
    restored=copy.deepcopy(root);restored.find('world/physics/max_step_size').text=previous
    if ET.tostring(restored)!=ET.tostring(ET.fromstring(original)):
        raise ValueError('step refinement changed another world element')
    proof_file=fixture/'native_fixture_proof.json'
    proof=json.loads(proof_file.read_text())
    calibration=fixture/'wrench_calibration.json';certificate=json.loads(calibration.read_text())
    for robot,digest in proof['generated_sha256'].items():
        if hashlib.sha256((fixture/robot).read_bytes()).hexdigest()!=digest:
            raise ValueError('baseline fixture changed before step refinement')
    diagnostic=dict(baseline_step_s=baseline,step_divisor=divisor,refined_step_s=baseline/divisor,
        baseline_world_sha256=hashlib.sha256(original).hexdigest(),
        refined_world_sha256=hashlib.sha256(changed).hexdigest(),only_max_step_size_changed=True,
        physical_models_contacts_controllers_unchanged=True,static_diagnostic_only=True,
        new_sensor_dynamic_calibration_verified=False,autonomous_yaw_qualification_issued=False)
    backup=fixture/'world_before_step_refinement.sdf'
    if backup.exists():raise ValueError('step refinement requires a fresh baseline fixture')
    backup.write_bytes(original);world.write_bytes(changed)
    proof['baseline_generated_sha256']=dict(proof['generated_sha256'])
    proof['generated_sha256']['world.sdf']=diagnostic['refined_world_sha256']
    proof['numerical_refinement']=diagnostic
    certificate.update(transfer_proof='same measured native sensor sign/axes and canonical URDF; physics step refined for static diagnostic only',
        numerical_refinement=diagnostic)
    calibration.write_text(json.dumps(certificate,indent=2)+'\n')
    proof_file.write_text(json.dumps(proof,indent=2)+'\n')
    return diagnostic
