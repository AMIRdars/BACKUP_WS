"""Transfer measured numerical-step sensor evidence, never yaw/transport proof."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from amir_rotation_integration_audit import canonical_model


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify(measurement,fixture):
    measurement,fixture=Path(measurement).resolve(),Path(fixture).resolve()
    result=json.loads((measurement/'result.json').read_text())
    if (result.get('passed') is not True or result.get('sensor_six_axis_remeasurement_passed') is not True
            or result.get('world_step_verified') is not True or result.get('verified_step_s')!=.002
            or result.get('hardware_motion_gate_open') is not False or result.get('native_transport_qualified') is not False):
        raise ValueError('completed measured 2 ms six-axis sensor proof required')
    source=measurement/'generated'
    cert=json.loads((measurement/'calibration/calibration.json').read_text())
    if (cert.get('sensor_calibration_passed') is not True or cert.get('robot')!='follower'
            or set(cert.get('verified_axes',[]))!={'fx','fy','fz','tx','ty','tz'}
            or cert.get('urdf_sha256')!=digest(source/'follower.urdf')):
        raise ValueError('measured native six-axis certificate/model mismatch')
    if result.get('fixture_world_sha256')!=digest(source/'world.sdf'):
        raise ValueError('remeasured calibration world changed')
    for robot in ('leader','follower'):
        if canonical_model(source/(robot+'.urdf'))!=canonical_model(fixture/(robot+'.urdf')):
            raise ValueError('remeasured native robot differs beyond controller path')
        for suffix in ('_controllers.yaml','_base_controller.yaml'):
            if (source/(robot+suffix)).read_bytes()!=(fixture/(robot+suffix)).read_bytes():
                raise ValueError('remeasured native controller changed')
    calibration=ET.parse(source/'world.sdf').getroot();removed=[]
    for parent in calibration.iter():
        for child in list(parent):
            if child.tag=='plugin' and child.get('name')=='amir_follower::CalibrationWrench':
                removed.append(child);parent.remove(child)
    native=ET.parse(fixture/'world.sdf').getroot()
    if len(removed)!=1 or ET.tostring(calibration)!=ET.tostring(native):
        raise ValueError('operational world must match measured world with only calibration actuator removed')
    if float(native.findtext('world/physics/max_step_size'))!=.002:
        raise ValueError('measured candidate requires the actual 2 ms world')
    workspace=Path(__file__).resolve().parents[1]
    hashes=json.loads((measurement/'source_sha256.json').read_text())
    if not hashes:raise ValueError('remeasurement source snapshot missing')
    for name,expected in hashes.items():
        saved=measurement/'source_snapshot'/Path(name).relative_to(workspace)
        if not saved.is_file() or digest(saved)!=expected or digest(name)!=expected:
            raise ValueError('remeasured source changed: '+name)
    return dict(measurement_directory=str(measurement),result_sha256=digest(measurement/'result.json'),
        certificate_sha256=digest(measurement/'calibration/calibration.json'),verified_step_s=.002,
        canonical_native_models_identical=True,only_calibration_actuator_removed=True,
        passive_yaw_qualification_issued=False,native_transport_qualified=False)


def prepare(output,source,original_metrology,measurement):
    from amir_rotation_native_fixture import prepare as baseline_prepare
    from amir_rotation_native_step_fixture import refine
    output=Path(output).resolve()
    baseline_prepare(output,source,original_metrology)
    refine(output,2)
    proof=verify(measurement,output)
    cert=json.loads((Path(measurement)/'calibration/calibration.json').read_text())
    cert.update(urdf_sha256=digest(output/'follower.urdf'),measured_step_transfer=proof,
        transfer_proof='identical native canonical URDF/controllers and measured 2 ms world with only calibration actuator removed')
    (output/'wrench_calibration.json').write_text(json.dumps(cert,indent=2)+'\n')
    (output/'measured_step_proof.json').write_text(json.dumps(proof,indent=2)+'\n')
    native_proof=json.loads((output/'native_fixture_proof.json').read_text())
    native_proof['measured_step_transfer']=proof
    native_proof['transferred_sensor_certificate_sha256']=digest(output/'wrench_calibration.json')
    native_proof['numerical_refinement'].update(static_diagnostic_only=False,six_axis_remeasurement_verified=True)
    (output/'native_fixture_proof.json').write_text(json.dumps(native_proof,indent=2)+'\n')
    return proof


def require_certificate(measurement,fixture,certificate):
    proof=verify(measurement,fixture)
    current=json.loads(Path(certificate).read_text())
    original=json.loads((Path(measurement)/'calibration/calibration.json').read_text())
    if current.get('measured_step_transfer')!=proof or current.get('urdf_sha256')!=digest(Path(fixture)/'follower.urdf'):
        raise ValueError('measured 2 ms certificate transfer proof missing or changed')
    for name,value in original.items():
        if name not in ('urdf_sha256','transfer_proof') and current.get(name)!=value:
            raise ValueError('measured sensor mapping changed: '+name)
    return proof
