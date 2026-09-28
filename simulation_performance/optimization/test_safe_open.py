"""Verify that opening uses fresh measured positions, never action flags alone."""
import time
from types import SimpleNamespace as S
from cooperative_transport_control.dual_base_approach import DualBaseApproach
values={'state_timeout':.5,'gripper_open_position':-1.0,'gripper_open_tolerance':.02}
n=S(_names=('amir1','amir2'),_accepted_open_goals={'amir1','amir2'},
    _gripper_positions={'amir1':-1.,'amir2':-1.},
    _gripper_state_times={'amir1':time.monotonic(),'amir2':time.monotonic()},
    get_parameter=lambda key:S(value=values[key]))
assert DualBaseApproach._grippers_open(n)
n._gripper_positions['amir2']=-.5
assert not DualBaseApproach._grippers_open(n), 'closed stalled gripper must not pass'
n._gripper_positions['amir2']=-1.
n._gripper_state_times['amir2']=time.monotonic()-1
assert not DualBaseApproach._grippers_open(n), 'stale joint state must not pass'
n._gripper_state_times['amir2']=time.monotonic()
n._accepted_open_goals.remove('amir2')
assert not DualBaseApproach._grippers_open(n), 'unaccepted action must not pass'
n._phase='OPENING_GRIPPERS'
n.get_logger=lambda:S(info=lambda _:None)
DualBaseApproach._on_open_result(n,'amir2',S(result=lambda:S(result=S(reached_goal=False,stalled=True))))
assert n._phase=='OPENING_GRIPPERS', 'stalled result must not start moving'
print('PASS: physical opening, freshness, accepted goal, and stalled/canceled result gates')
