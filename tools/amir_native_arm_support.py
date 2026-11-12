"""Native CP-13 grasp assets/preparation and measured position-servo probe."""
import hashlib
import json
import subprocess
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import re
import time
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python
from rclpy.time import Time
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from amir_compliance_control.arm_servo_response import model_digest,validate_response
from amir_compliance_control.stroke_kinematics import solve_stroke_pose as solve_pose
from amir_compliance_control.wrench import LocalModel,origin

SCENE=Path('audits/whole_body_compliance_20261019/cp12_speed_release/generated_gui20')
METROLOGY=Path('audits/whole_body_compliance_20261019/cp9_transport/metrology_02')


def controller_isolation_evidence(prefix=None):
    """Require the same-version JTC race fix before native two-arm motion."""
    prefix=Path(prefix or get_package_prefix('joint_trajectory_controller')).resolve()
    marker=prefix/'share/joint_trajectory_controller/AMIR_PATCH.json'
    library=prefix/'lib/libjoint_trajectory_controller.so'
    if not marker.is_file() or not library.is_file():
        raise ValueError('native ARM_ONLY requires the workspace JTC 2.53.1 isolation patch; '
                         'build joint_trajectory_controller and source install/setup.bash')
    proof=json.loads(marker.read_text())
    version=ET.parse(prefix/'share/joint_trajectory_controller/package.xml').findtext('version')
    if proof.get('version')!='2.53.1' or version!='2.53.1':
        raise ValueError('native ARM_ONLY requires the audited JTC 2.53.1 version')
    return {'prefix':str(prefix),'library':str(library.resolve()),
            'library_sha256':hashlib.sha256(library.read_bytes()).hexdigest(),
            'provenance':proof}


def verify_loaded_controller(proof, partition):
    """Evaluator-only /proc audit: the scene actually loaded the qualified DSO."""
    loaded=[]
    for process in Path('/proc').iterdir():
        if not process.name.isdigit():continue
        try:
            if ('IGN_PARTITION='+partition).encode() not in (process/'environ').read_bytes().split(b'\0'):continue
            paths={line.split()[-1] for line in (process/'maps').read_text().splitlines()
                   if 'libjoint_trajectory_controller.so' in line}
        except (OSError,UnicodeError):continue
        for path in paths:
            if str(Path(path).resolve())!=proof['library']:
                raise ValueError('scene loaded an unqualified JTC library: '+path)
            loaded.append({'pid':int(process.name),'library':path})
    if not loaded:raise ValueError('no loaded JTC library found in the isolated scene')
    return {**proof,'loaded_processes':loaded}


def wait_native_keep_ready(wait,monitor,maintenance,status,states,monitor_wall,clock=time.monotonic):
    """Same one-second unarmed readiness contract as CP13, explicit keeper ACK."""
    stable=[None]
    def ready():
        now=clock()
        good=bool(status.get('ready')) and all(
            monitor.get(r,{}).get('state')=='HOLDING' and monitor[r].get('healthy') is True
            and monitor[r].get('maintenance_ready') is True and now-monitor_wall.get(r,0.)<.25
            and maintenance.get(r,{}).get('enabled') is True
            and states.get(r,{}).get('local_ready') for r in ('leader','follower'))
        stable[0]=now if good and stable[0] is None else stable[0] if good else None
        return stable[0] is not None and now-stable[0]>=1.
    wait(ready,30.,True)


def base_envelope(xml):
    """Conservative chassis/wheel XY circle from actual collision meshes."""
    root=ET.fromstring(xml);parents={j.find('child').get('link'):j for j in root.findall('joint')}
    def transform(link):
        if link=='base_footprint':return np.eye(4)
        joint=parents[link];return transform(joint.find('parent').get('link'))@origin(joint.find('origin'))
    bound=0.;evidence=[]
    names=['base_link',*[root.find(f"joint[@name='{side}_joint']/child").get('link')
                            for side in ('front_left','front_right','back_left','back_right')]]
    for name in names:
        for collision in root.find(f"link[@name='{name}']").findall('collision'):
            mesh=collision.find('geometry/mesh')
            if mesh is None:
                geometry=collision.find('geometry');box=geometry.find('box');cylinder=geometry.find('cylinder');sphere=geometry.find('sphere')
                if box is not None:local_radius=float(np.linalg.norm([float(v)/2 for v in box.get('size').split()]))
                elif cylinder is not None:local_radius=float(np.hypot(float(cylinder.get('radius')),float(cylinder.get('length'))/2))
                elif sphere is not None:local_radius=float(sphere.get('radius'))
                else:raise ValueError('unsupported base collision geometry')
                t=transform(name)@origin(collision.find('origin'))
                radius=float(np.linalg.norm(t[:2,3])+local_radius);bound=max(bound,radius)
                evidence.append({'link':name,'radius_bound_m':radius,'primitive':ET.tostring(geometry,encoding='unicode')})
                continue
            uri=mesh.get('filename');package,relative=uri.removeprefix('package://').split('/',1)
            path=Path(get_package_share_directory(package))/relative;data=path.read_bytes()
            n=int.from_bytes(data[80:84],'little') if len(data)>=84 else 0
            if len(data)==84+50*n:
                dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
                vertices=np.frombuffer(data,offset=84,dtype=dtype,count=n)['vertices'].reshape(-1,3).astype(float)
            else:
                vertices=np.array([[float(v) for v in row] for row in re.findall(rb'vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)',data)])
            vertices*=np.array([float(v) for v in mesh.get('scale','1 1 1').split()])
            t=transform(name)@origin(collision.find('origin'))
            radius=float(np.linalg.norm(t[:2,3])+np.max(np.linalg.norm(vertices,axis=1)))
            bound=max(bound,radius);evidence.append({'link':name,'radius_bound_m':radius,'mesh_sha256':hashlib.sha256(data).hexdigest()})
    return {'xy_radius_bound_m':bound,'meshes':evidence,'scope':'chassis and wheels only; conservative circle, not whole-arm collision proof'}


def copy_native(root,folder,base_fixture=False):
    original=root/SCENE;proof={}
    names=['world.sdf','scene_staging.json','display_only_equivalence.json']
    names += [r+s for r in ('leader','follower') for s in ('.urdf','_controllers.yaml','_base_controller.yaml','_safety.json')]
    for name in names:
        (folder/name).write_bytes((original/name).read_bytes())
        proof[name]=hashlib.sha256((folder/name).read_bytes()).hexdigest()
    cert=json.loads((root/METROLOGY/'calibration/calibration.json').read_text())
    if cert['urdf_sha256']!=proof['follower.urdf']:raise ValueError('native F/T certificate mismatch')
    for robot in ('leader','follower'):
        profile=json.loads((folder/(robot+'_safety.json')).read_text())
        if profile['urdf_sha256']!=proof[robot+'.urdf']:
            raise ValueError('native safety evidence mismatch')
        for kind in ('controller','base_controller'):
            path=Path(profile[kind+'_yaml_file'])
            if hashlib.sha256(path.read_bytes()).hexdigest()!=profile[kind+'_yaml_sha256']:
                raise ValueError('native referenced controller changed: '+str(path))
    display=json.loads((folder/'display_only_equivalence.json').read_text())
    if display.get('passed') is not True or not display.get('all_other_world_xml_equal'):
        raise ValueError('native display-only equivalence proof required')
    if base_fixture:
        world=ET.parse(folder/'world.sdf')
        plugin=ET.SubElement(world.getroot().find('world'),'plugin',
            filename=str(Path(get_package_prefix('amir_follower_gazebo'))/'lib/libamir_base_hold_fixture-system.so'),
            name='amir_follower::BaseHoldFixture')
        ET.SubElement(plugin,'robot').text='follower';ET.SubElement(plugin,'base_link').text='base_footprint'
        world.write(folder/'world.sdf',encoding='unicode',xml_declaration=True)
        fixture_proof={'source_world':str(original/'world.sdf'),'source_world_sha256':proof['world.sdf'],
            'only_added_plugin':'amir_follower::BaseHoldFixture','initially_locked':False,
            'plugin_library_sha256':hashlib.sha256(Path(plugin.get('filename')).read_bytes()).hexdigest(),
            'fixture_method':'post-grasp root pose restraint through local root velocities; no model pose commands or articulated joint reset',
            'scope':'simulation test base fixture only; native object/gripper contact remains frictional'}
        (folder/'base_fixture_proof.json').write_text(json.dumps(fixture_proof,indent=2))
        display.update(all_other_world_xml_equal=False,base_fixture_only_difference=True)
        (folder/'display_only_equivalence.json').write_text(json.dumps(display,indent=2))
        validate_native_scene(folder)
    # Same derivation used by the public CP13 trial. Derive for this copied
    # world from measured native joint loads; do not patch a stale hash alone.
    derived=subprocess.run(['/usr/bin/python3',str(root/'tools/amir_make_safety_profiles.py'),
        '--probe',str(root/METROLOGY/'static_proof/runtime.json'),'--run-directory',str(folder),
        '--motion-evidence',str(root/'audits/whole_body_compliance_20261019/cp1/motion_run3/motion.json')],
        check=True,capture_output=True,text=True)
    (folder/'safety_profile_derivation.json').write_text(derived.stdout)
    (folder/'calibration.json').write_text(json.dumps(cert,indent=2))
    (folder/'fixture_relocation.json').write_text(json.dumps({'source':str(original),
        'source_native_assets_sha256':proof,'fixed_grasp':False,'base_fixture':base_fixture,
        'world_sha256':hashlib.sha256((folder/'world.sdf').read_bytes()).hexdigest()},indent=2))
    envelopes={r:base_envelope((folder/(r+'.urdf')).read_text()) for r in ('leader','follower')}
    (folder/'base_collision_envelopes.json').write_text(json.dumps(envelopes,indent=2))


def validate_native_scene(folder):
    proof=json.loads((folder/'base_fixture_proof.json').read_text())
    source=Path(proof['source_world'])
    if hashlib.sha256(source.read_bytes()).hexdigest()!=proof['source_world_sha256']:
        raise ValueError('native source world changed')
    actual=ET.parse(folder/'world.sdf');world=actual.getroot().find('world')
    plugins=world.findall("plugin[@name='amir_follower::BaseHoldFixture']")
    if len(plugins)!=1:raise ValueError('one explicit base fixture plugin required')
    plugin=plugins[0]
    if plugin.findtext('robot')!='follower' or plugin.findtext('base_link')!='base_footprint':
        raise ValueError('base fixture target changed')
    if hashlib.sha256(Path(plugin.get('filename')).read_bytes()).hexdigest()!=proof['plugin_library_sha256']:
        raise ValueError('base fixture library changed')
    world.remove(plugin)
    # Ignore indentation only. All physical values and native plugins must match.
    def clean(element):
        element.text=(element.text or '').strip();element.tail=''
        for child in element:clean(child)
    original=ET.parse(source);clean(original.getroot());clean(actual.getroot())
    if ET.tostring(original.getroot())!=ET.tostring(actual.getroot()):
        raise ValueError('unexpected native world change beyond base fixture')


def stroke_plan(xml,distance):
    if not np.isfinite(distance) or not 0<distance<=.5:raise ValueError('native leader stroke must be within (0,0.5] m')
    root=ET.fromstring(xml);model=LocalModel(xml)
    q=np.array([float(root.find(f"ros2_control/joint[@name='{n}']/state_interface[@name='position']/param").text) for n in model.names])
    home,sensor=model.frames(q);rotation=sensor[:3,:3];samples=[]
    for x in np.linspace(0,distance,101):
        target=home-np.array([.5*x,0.,0.]);q=solve_pose(model,q,target,rotation)
        sigma=float(np.linalg.svd(model.jacobian(q)[:2],compute_uv=False)[-1])
        samples.append({'leader_base_m':float(x),'arm_retraction_m':float(.5*x),'q_rad':q.tolist(),
            'min_joint_margin_rad':float(np.min(np.minimum(q-model.lower,model.upper-q))),
            'sigma_min_xy_m':sigma})
    return {'passed':True,'leader_base_distance_m':distance,'nominal_payload_distance_m':distance*.5,
        'nominal_each_arm_retraction_m':distance*.5,'samples':samples,
        'scope':'continuous same-orientation IK branch only; collision/load checked in simulation'}


def servo_probe_ready(latest,jtc_samples,homes,pubs):
    for robot,home in homes.items():
        if (robot,'q') not in latest or not jtc_samples[robot] or pubs[robot].get_subscription_count()<1:return False
        msg=latest[robot,'q'];positions=dict(zip(msg.name,msg.position));speeds=dict(zip(msg.name,msg.velocity))
        if not all(name in positions and name in speeds for name in (f'Joint_{i}' for i in range(1,6))):return False
        q=np.array([positions[f'Joint_{i}'] for i in range(1,6)])
        v=np.array([speeds[f'Joint_{i}'] for i in range(1,6)])
        # First one-second probe excursion is 2 mrad. Its worst commanded
        # ramp from the actual starting position must fit the existing .03
        # rad/s experiment envelope. Do not home a zero-state arm at probe speed.
        if not np.all(np.isfinite(np.r_[q,v])) or np.max(abs(q-home))+.002>.03 or np.max(abs(v))>.001:return False
    return True


def measure_servo(*,node,folder,out,latest,jtc_samples,sim,wait,simwait,phase,event):
    """Before grasp, support retained. Sole temporary arm writer, then release."""
    pubs={r:node.create_publisher(JointTrajectory,'/'+r+'/arm_controller/joint_trajectory',1) for r in ('leader','follower')}
    homes={}
    for r in pubs:
        root=ET.parse(folder/(r+'.urdf'));homes[r]=[float(root.find(f"ros2_control/joint[@name='Joint_{i}']/state_interface[@name='position']/param").text) for i in range(1,6)]
    def send(offset):
        for r,pub in pubs.items():
            q=(np.array(homes[r])+offset).tolist();msg=JointTrajectory(joint_names=[f'Joint_{i}' for i in range(1,6)])
            msg.header.stamp=Time(seconds=sim[0]).to_msg()
            point=JointTrajectoryPoint(positions=q);point.time_from_start.sec=1;msg.points=[point];pub.publish(msg)
    try:
        phase[0]='servo_probe_readiness'
        def ready():
            # The experiment recorder is also a subscriber. Confirm the real
            # controller endpoint by identity instead of mistaking subscriber
            # count for the single-command-writer requirement.
            connected=all(any(info.node_name=='arm_controller' and info.node_namespace=='/'+robot
                for info in node.get_subscriptions_info_by_topic('/'+robot+'/arm_controller/joint_trajectory')) for robot in pubs)
            return connected and servo_probe_ready(latest,jtc_samples,homes,pubs)
        wait(ready,30.)
        phase[0]='servo_probe'
        for offset in (np.full(5,.002),np.full(5,-.002),np.full(5,.002),np.zeros(5)):
            send(offset);simwait(1.5,False)
        wait(lambda:all(max(abs(a-b) for a,b in zip(
            [dict(zip(latest[r,'q'].name,latest[r,'q'].position))[f'Joint_{i}'] for i in range(1,6)],homes[r]))<1e-5 for r in pubs),10.)
    finally:
        for pub in pubs.values():node.destroy_publisher(pub)
    for r in pubs:
        params={}
        for name,keys in [('gz_ros2_control',['position_proportional_gain']),('controller_manager',['update_rate']),('arm_controller',['open_loop_control'])]:
            client=node.create_client(GetParameters,f'/{r}/{name}/get_parameters');wait(client.service_is_ready,10.)
            future=client.call_async(GetParameters.Request(names=keys));wait(future.done,5.)
            params[name]={k:parameter_value_to_python(v) for k,v in zip(keys,future.result().values)};node.destroy_client(client)
        gain=params['gz_ros2_control']['position_proportional_gain'];rate=params['controller_manager']['update_rate']
        rows=jtc_samples[r];names=[f'Joint_{i}' for i in range(1,6)];response={};tau=[]
        for i,name in enumerate(names):
            valid=[v for v in rows if v['phase']=='servo_probe' and len(v['feedback_positions'])==5 and len(v['feedback_velocities'])==5]
            error=np.array([(v['output_positions'] if len(v['output_positions'])==5 else v['reference_positions'])[i]-v['feedback_positions'][i] for v in valid])
            speed=np.array([v['feedback_velocities'][i] for v in valid]);mask=(abs(error)>2e-6)&(abs(speed)>1e-5)
            count=int(mask.sum());fitted=float(error[mask]@speed[mask]/(error[mask]@error[mask])) if count else None
            verified=count>=20 and abs(fitted/(gain*rate)-1)<=.15
            response[name]={'samples':count,'velocity_gain_s_inv':fitted,'verified':bool(verified)}
            tau.append(1/(gain*rate) if verified else 0.)
        cert={'scope':'simulation_position_servo_response_only','passed':sum(t>0 for t in tau)>=3,'ungrasped':True,
            'joint_names':names,'normalized_urdf_sha256':model_digest((folder/(r+'.urdf')).read_text(),folder),
            'position_proportional_gain':gain,'controller_update_rate_hz':rate,'joint_response':response,
            'joint_compensation_s':tau,'open_loop_control':params['arm_controller']['open_loop_control'],
            'plugin_library_sha256':hashlib.sha256(Path('/opt/ros/humble/lib/libgz_hardware_plugins.so').read_bytes()).hexdigest()}
        validate_response(cert,(folder/(r+'.urdf')).read_text(),folder)
        (out/(r+'_servo_response.json')).write_text(json.dumps(cert,indent=2))
    event('native_servo_probe','both',{'ungrasped':True,'support_retained':True,'temporary_writers_released':True})
