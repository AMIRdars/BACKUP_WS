"""Generate a NEW native fixture and prove calibration transfer before launch.

Only the controller-file location may differ in the robot descriptions.
Native world geometry, contact policy and controller bytes must match the
native calibration source. No fixed-grasp certificate is accepted here.
"""
import hashlib,importlib.util,json
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from amir_rotation_integration_audit import canonical_model


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(output,source,metrology):
 output=output.resolve();source=source.resolve();metrology=metrology.resolve()
 output.mkdir(parents=True,exist_ok=False)
 certificate=json.loads((metrology/'calibration/calibration.json').read_text())
 if certificate.get('sensor_calibration_passed') is not True or certificate['urdf_sha256']!=sha(source/'follower.urdf'):
  raise ValueError('matching native measured F/T certificate required')
 path=Path(get_package_share_directory('cooperative_transport_bringup'))/'launch/compliance_cp9_contact.launch.py'
 spec=importlib.util.spec_from_file_location('native_contact_audit_generator',path)
 module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
 context=LaunchContext();context.launch_configurations['run_directory']=str(output)
 for entity in module.generate_launch_description().entities:
  if isinstance(entity,DeclareLaunchArgument) and entity.name!='run_directory':
   context.launch_configurations[entity.name]=perform_substitutions(context,entity.default_value)
 module.setup(context)  # Generate actions/files only; never execute a launch.
 files=['world.sdf',*[r+s for r in ('leader','follower') for s in ('.urdf','_controllers.yaml','_base_controller.yaml')]]
 hashes={name:sha(output/name) for name in files}
 module.setup(context)
 if hashes!={name:sha(output/name) for name in files}:raise ValueError('native fixture generation is not idempotent')
 for robot in ('leader','follower'):
  if canonical_model(source/(robot+'.urdf'))!=canonical_model(output/(robot+'.urdf')):
   raise ValueError('native robot model changed beyond controller path')
  for suffix in ('_controllers.yaml','_base_controller.yaml'):
   if (source/(robot+suffix)).read_bytes()!=(output/(robot+suffix)).read_bytes():
    raise ValueError('native controller parameters changed')
  # The existing trial regenerates these from native measured metrology
  # before arming. Keep an initial native-only profile available to it.
  (output/(robot+'_safety.json')).write_bytes((source/(robot+'_safety.json')).read_bytes())
 if ET.tostring(ET.parse(source/'world.sdf').getroot())!=ET.tostring(ET.parse(output/'world.sdf').getroot()):
  raise ValueError('native physical world/contact policy changed')
 allowed={'Physics','UserCommands','SceneBroadcaster','Contact','Sensors','ForceTorque','DartContactParameters'}
 plugins=[e.get('name','').rsplit('::',1)[-1] for e in ET.parse(output/'world.sdf').findall('.//plugin')]
 if not set(plugins)<=allowed:raise ValueError('native fixture contains an assisted grasp or calibration actuator')
 certificate.update(urdf_sha256=hashes['follower.urdf'],transferred_from=str(metrology/'calibration/calibration.json'),
  transfer_proof='native canonical URDF equality except controller path; identical world and controller bytes')
 calibration=output/'wrench_calibration.json';calibration.write_text(json.dumps(certificate,indent=2)+'\n')
 report=dict(native_contact_only=True,source_native_fixture=str(source),source_native_metrology=str(metrology),
  generator_sha256=sha(path),source_models_sha256={r:sha(source/(r+'.urdf')) for r in ('leader','follower')},
  generated_sha256=hashes,generation_idempotent=True,plugins=plugins,certificate_file=str(calibration),
  source_sensor_certificate_sha256=sha(metrology/'calibration/calibration.json'),autonomous_yaw_qualification_issued=False)
 (output/'native_fixture_proof.json').write_text(json.dumps(report,indent=2)+'\n');return report
