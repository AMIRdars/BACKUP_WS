import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import pytest

from amir_rotation_native_measured_step import prepare,require_certificate

WORKSPACE=Path(__file__).resolve().parents[1]
NATIVE=WORKSPACE/'audits/object_center_rotation_20261103/cp8r3'
MEASURED=WORKSPACE/'audits/rotation_native_enable_20261106/native_metrology_step2_01'


def fixture(tmp_path):
    output=tmp_path/'native'
    proof=prepare(output,NATIVE/'native_generated',NATIVE/'native_metrology',MEASURED)
    return output,proof


def test_only_calibration_actuator_is_removed_and_new_sensor_mapping_is_preserved(tmp_path):
    output,proof=fixture(tmp_path)
    assert require_certificate(MEASURED,output,output/'wrench_calibration.json')==proof
    world=ET.parse(output/'world.sdf')
    assert world.findtext('world/physics/max_step_size')=='0.002'
    assert not any(p.get('name')=='amir_follower::CalibrationWrench' for p in world.findall('.//plugin'))
    assert proof['passive_yaw_qualification_issued'] is False and proof['native_transport_qualified'] is False


@pytest.mark.parametrize('failure',['sign','certificate_proof','world','controller','model'])
def test_changed_mapping_or_native_fixture_cannot_use_measured_step_proof(tmp_path,failure):
    output,_=fixture(tmp_path)
    if failure in ('sign','certificate_proof'):
        p=output/'wrench_calibration.json';value=json.loads(p.read_text())
        if failure=='sign':value['sign']=-value['sign']
        else:value.pop('measured_step_transfer')
        p.write_text(json.dumps(value))
    elif failure=='world':
        p=output/'world.sdf';tree=ET.parse(p);tree.find('world/gravity').text='0 0 -9.8';tree.write(p)
    elif failure=='controller':(output/'follower_controllers.yaml').write_text('changed')
    else:
        p=output/'follower.urdf';p.write_text(p.read_text().replace('name="tcp_link"','name="changed_tcp"'))
    with pytest.raises(ValueError):require_certificate(MEASURED,output,output/'wrench_calibration.json')
