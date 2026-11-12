"""Early safety stops cannot be represented as a complete applied pulse."""
from pathlib import Path
import runpy

import numpy as np
import pytest


input_end_metrics=runpy.run_path(str(Path(__file__).parents[1]/'experiments/analyze_control.py'))['input_end_metrics']


@pytest.mark.parametrize('input_end,covered',[(1.25,True),(3.,False),(-1.,False)])
def test_input_end_requires_bracketing_measurements(input_end,covered):
    result=input_end_metrics(np.array([0.,1.,2.]),np.array([[0.,0.,0.],[.1,.2,.3],[.2,.4,.6]]),
                             np.array([0.,.01,.02]),input_end)
    assert result['input_end_covered_by_log'] is covered
    if covered:
        assert result['input_end_sample_sim_time']==1.
        assert result['actual_G_B_delta_at_input_end_m']==[.1,.2,.3]
        assert result['spatial_omega_z_integral_at_input_end_rad']==.01
    else:
        assert result['input_end_sample_sim_time'] is None
        assert result['actual_G_B_delta_at_input_end_m'] is None
        assert result['spatial_omega_z_integral_at_input_end_rad'] is None
