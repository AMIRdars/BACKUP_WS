"""Meaningful bounds and unilateral/overload cases for simulated grasp setup."""
import importlib.util
from pathlib import Path
import pytest
import xml.etree.ElementTree as ET


URDF=Path(__file__).resolve().parent.parent/'audits/whole_body_compliance_20261019/cp6/generated/leader.urdf'


def test_home_is_from_actual_model_not_a_new_numeric_pose():
    home=grasp.home_from_urdf(URDF)
    root=ET.parse(URDF).getroot()
    assert len(home)==5
    for name,value in home.items():
        assert value==float(root.find("ros2_control/joint[@name='"+name+"']/state_interface[@name='position']/param").text)


@pytest.mark.parametrize('change',['hardware','missing_hardware','nan','outside','missing_joint'])
def test_bad_home_and_hardware_models_rejected(tmp_path,change):
    root=ET.parse(URDF).getroot()
    if change=='hardware':root.find('ros2_control/hardware/plugin').text='amir_driver/AmirHardwareInterfaces'
    elif change=='missing_hardware':
        for system in root.findall('ros2_control'):system.remove(system.find('hardware'))
    elif change in ('nan','outside'):
        root.find("ros2_control/joint[@name='Joint_1']/state_interface[@name='position']/param").text='nan' if change=='nan' else '100'
    else:
        for system in root.findall('ros2_control'):
            joint=system.find("joint[@name='Joint_1']")
            if joint is not None:system.remove(joint)
    path=tmp_path/'model.urdf';path.write_text(ET.tostring(root,encoding='unicode'))
    with pytest.raises(ValueError):grasp.home_from_urdf(path)

spec=importlib.util.spec_from_file_location('grasp',Path(__file__).with_name('amir_cp9_local_grasp.py'))
grasp=importlib.util.module_from_spec(spec);spec.loader.exec_module(grasp)


def test_unilateral_overforce_releases_instead_of_searching():
    assert grasp.position_step(-.6,[30.,0.],False,False)<-.6


def test_contact_reacquisition_is_finer_than_first_search():
    first=grasp.position_step(-.6,[0.,0.],False,False)+.6
    reacquire=grasp.position_step(-.6,[0.,0.],False,True)+.6
    assert 0<reacquire<first


def test_both_fingers_must_be_in_target_band():
    assert grasp.position_step(-.6,[16.,22.],True,True)>-.6
    assert grasp.position_step(-.6,[20.,20.],True,True)==-.6


@pytest.mark.parametrize('q',[-1.,-.6,.261799])
@pytest.mark.parametrize('forces',[[0.,0.],[20.,20.],[100.,0.]])
def test_joint_and_step_bounds(q,forces):
    out=grasp.position_step(q,forces,True,True)
    assert -1<=out<=.261799 and abs(out-q)<=.006+1e-12


@pytest.mark.parametrize('forces',[[float('nan'),1.],[1.,float('inf')],[-1.,0.],[1.]])
def test_invalid_force_rejected(forces):
    with pytest.raises(ValueError):grasp.position_step(-.6,forces,False,False)


@pytest.mark.parametrize('q',[float('nan'),-2.,.4])
def test_invalid_joint_rejected(q):
    with pytest.raises(ValueError):grasp.position_step(q,[0.,0.],False,False)


def test_recovering_from_below_nominal_open_is_still_step_bounded():
    q=-1.02;result=grasp.position_step(q,[0.,0.],False,False)
    assert q<result<=q+.002+1e-12


@pytest.mark.parametrize('kind',['healthy','missing','nan','range','tracking','offset','no_specs'])
def test_measured_mimic_limits_and_relation(kind):
    specs={'finger':{'lower':-1.047198,'upper':.261799,'multiplier':1.,'offset':0.}}
    values={'finger':-.6}
    if kind=='missing':values={}
    if kind=='nan':values['finger']=float('nan')
    if kind=='range':values['finger']=-1.052
    if kind=='tracking':values['finger']=-.5
    if kind=='offset':specs['finger']['offset']=.1;values['finger']=-.5
    if kind=='no_specs':specs={};values={}
    assert grasp.mimic_measurement_ok(-.6,values,specs)==(kind in ('healthy','offset'))
