"""Real-URDF half-stroke feasibility, residuals and preserved native assets."""
import json
from pathlib import Path
import numpy as np
import pytest
from amir_native_arm_support import stroke_plan,SCENE,copy_native,validate_native_scene,base_envelope,wait_native_keep_ready,servo_probe_ready
import xml.etree.ElementTree as ET
from amir_compliance_control.wrench import LocalModel
from amir_compliance_control.admittance import rotation_error
from amir_compliance_control.stroke_kinematics import solve_stroke_pose
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument,ExecuteProcess
from launch.utilities import perform_substitutions
import runpy

ROOT=Path(__file__).resolve().parents[1]


def test_native_motion_requires_same_version_isolated_controller(tmp_path):
    from amir_native_arm_support import controller_isolation_evidence
    with pytest.raises(ValueError,match='isolation patch'):
        controller_isolation_evidence(tmp_path)
    proof=controller_isolation_evidence()
    assert proof['provenance']['version']=='2.53.1'
    assert len(proof['library_sha256'])==64


def model():
    xml=(ROOT/SCENE/'follower.urdf').read_text()
    return xml,LocalModel(xml)


def test_half_metre_base_path_with_quarter_metre_arm_stays_in_joint_and_task_margin():
    xml,m=model();plan=stroke_plan(xml,.5)
    home,frame=m.frames(plan['samples'][0]['q_rad'])
    for sample in plan['samples']:
        p,s=m.frames(sample['q_rad']);target=home-[sample['arm_retraction_m'],0.,0.]
        assert np.linalg.norm(p-target)<1e-6
        assert np.linalg.norm(rotation_error(s[:3,:3],frame[:3,:3]))<1e-5
        assert sample['min_joint_margin_rad']>.069
        assert sample['sigma_min_xy_m']>.1
    q=np.array([s['q_rad'] for s in plan['samples']]);dt=(.5/100)/.005
    assert np.max(np.abs(np.diff(q,axis=0)/dt))<.03
    assert plan['nominal_payload_distance_m']==.25


def test_uncompensated_half_metre_pose_is_not_falsely_accepted():
    _,m=model();q=np.array([0.,1.4,-1.6,.2,1.570796]);p,t=m.frames(q)
    with pytest.raises(ValueError):solve_stroke_pose(m,q,p-[.5,0.,0.],t[:3,:3])


@pytest.mark.parametrize('d',[0.,-.5,.51,float('nan')])
def test_unverified_distance_refused(d):
    xml,_=model()
    with pytest.raises(ValueError):stroke_plan(xml,d)


@pytest.mark.parametrize('headless,review',[('true','paused'),('false','paused'),('false','live')])
def test_public_native_branch_retains_grasp_then_uses_explicit_half_metre_base_stroke(tmp_path,headless,review):
    module=runpy.run_path(str(ROOT/'src/humbleble_ws/cooperative_transport_bringup/launch/compliance_round_trip.launch.py'))
    ctx=LaunchContext()
    for action in module['generate_launch_description']().entities:
        if isinstance(action,DeclareLaunchArgument):action.execute(ctx)
    ctx.launch_configurations.update(workspace_directory=str(ROOT),output_directory=str(tmp_path/'out'),
        arm_only_trial='native_stroke',
        forward_distance_m='1.0',translation_velocity_m_s='0.1',headless=headless,gui_review_mode=review)
    process=next(x for x in module['setup'](ctx) if isinstance(x,ExecuteProcess))
    argv=[perform_substitutions(ctx,x) for x in process.cmd]
    assert '--native-arm-stroke' in argv
    assert '--follower-base-fixture' in argv
    assert argv[argv.index('--distance')+1]=='0.5'
    assert argv[argv.index('--speed')+1]=='0.005'
    assert ('--pause-gui-after-stop' in argv)==(headless=='false' and review=='paused')
    policy=json.loads((tmp_path/'out/launch_policy.json').read_text())
    assert policy['nominal_payload_distance_m']==.25
    assert not policy['legacy_motion_arguments_used_for_transport']


def test_real_base_mesh_envelope_is_separate_from_arm_reach():
    xml,_=model();envelope=base_envelope(xml)
    assert .2<envelope['xy_radius_bound_m']<.3
    assert len(envelope['meshes'])==5


def test_base_fixture_changes_only_one_explicit_world_plugin(tmp_path):
    copy_native(ROOT,tmp_path,True);validate_native_scene(tmp_path)
    assert (tmp_path/'follower.urdf').read_bytes()==(ROOT/SCENE/'follower.urdf').read_bytes()
    world=ET.parse(tmp_path/'world.sdf')
    assert not world.findall('.//joint')
    plugin=world.find("world/plugin[@name='amir_follower::BaseHoldFixture']")
    assert plugin is not None and plugin.findtext('robot')=='follower'
    assert json.loads((tmp_path/'base_fixture_proof.json').read_text())['initially_locked'] is False
    world.find('world/gravity').text='0 0 -1';world.write(tmp_path/'world.sdf')
    with pytest.raises(ValueError,match='unexpected native world change'):validate_native_scene(tmp_path)


def test_maintenance_handoff_waits_for_monitor_ack_before_arming():
    now=[1.];monitor={r:{'state':'HOLDING','healthy':True,'maintenance_ready':False} for r in ('leader','follower')}
    keep={r:{'enabled':True} for r in monitor};states={r:{'local_ready':True} for r in monitor};walls={r:1. for r in monitor}
    def wait(predicate,timeout,healthy):
        assert not predicate()  # a successful enable service alone is insufficient
        for m in monitor.values():m['maintenance_ready']=True
        for _ in range(30):
            for r in walls:walls[r]=now[0]
            if predicate():return
            now[0]+=.05
        pytest.fail('readiness ACK never became stable')
    wait_native_keep_ready(wait,monitor,keep,{'ready':True},states,walls,clock=lambda:now[0])
    assert now[0]>=2.


def test_servo_probe_waits_for_both_controllers_and_safe_initial_position():
    from types import SimpleNamespace
    homes={r:np.array([0.,1.4,-1.6,.2,1.570796]) for r in ('leader','follower')}
    names=[f'Joint_{i}' for i in range(1,6)]
    latest={(r,'q'):SimpleNamespace(name=names,position=q.copy(),velocity=[0.]*5) for r,q in homes.items()}
    jtc={r:[{}] for r in homes};counts={r:1 for r in homes}
    pubs={r:SimpleNamespace(get_subscription_count=lambda r=r:counts[r]) for r in homes}
    assert servo_probe_ready(latest,jtc,homes,pubs)
    counts['follower']=2;assert servo_probe_ready(latest,jtc,homes,pubs)  # recorder + controller readers
    counts['follower']=0;assert not servo_probe_ready(latest,jtc,homes,pubs)
    counts['follower']=1;latest['follower','q'].position=np.zeros(5)
    assert not servo_probe_ready(latest,jtc,homes,pubs)
    latest['follower','q'].position=homes['follower'];jtc['follower']=[]
    assert not servo_probe_ready(latest,jtc,homes,pubs)
