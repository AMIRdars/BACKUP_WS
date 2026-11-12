"""Response evidence cannot authorize an unmeasured joint or changed model."""
import copy
import numpy as np
import pytest
from amir_compliance_control.arm_servo_response import model_digest,validate_response


def certificate():
 names=[f'Joint_{i}' for i in range(1,6)]
 return {'scope':'simulation_position_servo_response_only','passed':True,'ungrasped':True,
  'joint_names':names,'normalized_urdf_sha256':model_digest('<robot path="/sample/run/a"/>','/sample/run'),
  'position_proportional_gain':.1,'controller_update_rate_hz':250,
  'joint_compensation_s':[.04,.04,.04,.04,0.],
  'joint_response':{n:{'samples':100 if i<4 else 0,'velocity_gain_s_inv':25. if i<4 else None} for i,n in enumerate(names)}}


def test_path_relocation_preserves_model_but_fifth_joint_stays_uncompensated():
 c=certificate();tau=validate_response(c,'<robot path="/new/run/a"/>','/new/run')
 np.testing.assert_array_equal(tau,[.04,.04,.04,.04,0.])


@pytest.mark.parametrize('bad',['model','ungrasped','status','gain','count','unexcited','formula','coverage'])
def test_invalid_measurement_proof_rejected(bad):
 c=certificate();xml='<robot path="/sample/run/a"/>'
 if bad=='model':xml='<robot path="/sample/run/different"/>'
 if bad=='ungrasped':c['ungrasped']=False
 if bad=='status':c['passed']=False
 if bad=='gain':c['joint_response']['Joint_1']['velocity_gain_s_inv']=60.
 if bad=='count':c['joint_response']['Joint_2']['samples']=2
 if bad=='unexcited':c['joint_compensation_s'][4]=.04
 if bad=='formula':c['joint_compensation_s'][1]=.05
 if bad=='coverage':c['joint_compensation_s']=[.04,.04,0.,0.,0.]
 with pytest.raises(ValueError):validate_response(c,xml,'/sample/run')
