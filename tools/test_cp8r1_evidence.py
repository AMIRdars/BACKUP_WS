"""Independent planar metrology and quaternion conventions for CP-8R-1."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from amir_cp8r1_analyze import virtual_center_drift
from amir_cp8r1_trial import pose_matrix


@pytest.mark.parametrize('scale', [1., .955])
def test_virtual_center_distinguishes_perfect_encoder_path_from_actual_yaw_slip(scale):
    radius, phi = 1.075, math.radians(5.)
    rows = [{'phase': 'baseline', 'leader_encoder_x': 0., 'leader_encoder_y': 0., 'leader_encoder_yaw': 0.,
             'leader_GT_x': -radius, 'leader_GT_y': 0., 'leader_GT_yaw': 0.},
            {'phase': 'hold', 'center_odom_x': radius, 'center_odom_y': 0.,
             'leader_GT_x': -radius*math.cos(phi), 'leader_GT_y': -radius*math.sin(phi), 'leader_GT_yaw': scale*phi}]
    assert virtual_center_drift(rows) == pytest.approx(2.*radius*math.sin((1.-scale)*phi/2.), abs=1e-15)


def test_evaluator_quaternion_maps_payload_original_y_axis_to_vertical():
    p = SimpleNamespace(x=0., y=0., z=.66)
    q = SimpleNamespace(x=math.sin(math.pi/4), y=0., z=0., w=math.cos(math.pi/4))
    t = pose_matrix(p, q)
    np.testing.assert_allclose(t[:3, 1], [0., 0., 1.], atol=1e-15)
    np.testing.assert_allclose(t[:3, 3], [0., 0., .66])
    q.w = 4.
    with pytest.raises(ValueError):
        pose_matrix(p, q)
