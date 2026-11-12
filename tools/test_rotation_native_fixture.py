"""A new fixture gets native sensor evidence only after model/world equality."""
import json,shutil
from pathlib import Path
import pytest
from amir_rotation_native_fixture import prepare,sha
ROOT=Path(__file__).resolve().parents[1]/'audits/object_center_rotation_20261103/cp8r3'


def test_native_fixture_generation_is_repeatable_and_preserves_contact_physics(tmp_path):
 p=tmp_path/'fresh';result=prepare(p,ROOT/'native_generated',ROOT/'native_metrology')
 assert result['native_contact_only'] and result['generation_idempotent']
 assert not result['autonomous_yaw_qualification_issued']
 cert=json.loads(Path(result['certificate_file']).read_text())
 assert cert['urdf_sha256']==sha(p/'follower.urdf')
 assert 'identical world' in cert['transfer_proof']
 assert 'GraspHeartbeat' not in result['plugins']


def test_changed_native_world_cannot_receive_a_transferred_certificate(tmp_path):
 source=tmp_path/'changed';shutil.copytree(ROOT/'native_generated',source)
 world=source/'world.sdf';world.write_text(world.read_text()+'<!-- different unverified native world -->')
 # Change physical gravity; comments alone are irrelevant to XML equality.
 import xml.etree.ElementTree as ET
 tree=ET.parse(world);tree.getroot().find('world/gravity').text='0 0 -9.80';tree.write(world)
 with pytest.raises(ValueError,match='world/contact policy changed'):
  prepare(tmp_path/'refused',source,ROOT/'native_metrology')
 assert not (tmp_path/'refused/wrench_calibration.json').exists()
