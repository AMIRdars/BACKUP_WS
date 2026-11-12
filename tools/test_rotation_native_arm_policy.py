"""Synthetic policy values exercise provenance; never create native evidence."""
import copy

import pytest

from amir_rotation_native_arm_policy import ARRAYS, SCALARS, validate, transfer


def unit_policy():
    return {**{name:[1.]*size for name,size in ARRAYS.items()}, **{name:1. for name in SCALARS},
        'rotational_relief_enabled':False,'arm_yaw_admittance_enabled':True,
        'native_grasp_monitor_required':True,'allow_ungrasped_test':False}


def test_transfer_uses_recorded_live_mechanics_instead_of_downstream_defaults():
    recorded=unit_policy();params={'rotational_relief_enabled':True,'virtual_yaw_inertia':999.}
    assert transfer({'native_arm_policy':recorded},{'native_arm_policy':recorded},params)==recorded
    assert params==recorded


@pytest.mark.parametrize('failure',['missing','nan','shape','bool','ungrasped','certificate'])
def test_missing_changed_or_nonfinite_arm_evidence_is_refused(failure):
    actual=unit_policy();certificate=copy.deepcopy(actual)
    if failure=='missing':actual.pop('relief_damping')
    elif failure=='nan':actual['virtual_yaw_damping']=float('nan')
    elif failure=='shape':actual['virtual_mass']=[1.]
    elif failure=='bool':actual['rotational_relief_enabled']=0
    elif failure=='ungrasped':actual['allow_ungrasped_test']=True
    else:certificate['rotational_relief_enabled']=True
    with pytest.raises(ValueError):transfer({'native_arm_policy':actual},{'native_arm_policy':certificate},{})
