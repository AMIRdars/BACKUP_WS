"""Offline simulation distance metrology; never consumes live ground truth."""
import copy
import hashlib
import math
from pathlib import Path


def validate_distance_calibration(cert, folder):
    if (cert.get('schema')!=1 or cert.get('passed') is not True or
            cert.get('scope')!='simulation_native_forward_progressive_stroke_distance_only'):
        raise ValueError('audited simulation base distance calibration required')
    names={'leader.urdf','leader_controllers.yaml','leader_base_controller.yaml','world.sdf','base_fixture_proof.json'}
    if set(cert.get('artifact_sha256',{}))!=names:raise ValueError('distance calibration artifact coverage')
    for name in names:
        if hashlib.sha256((Path(folder)/name).read_bytes()).hexdigest()!=cert['artifact_sha256'][name]:
            raise ValueError('distance calibration model/controller/world mismatch: '+name)
    samples=cert.get('samples',[])
    if len(samples)<2 or len({s['case'] for s in samples})!=len(samples):
        raise ValueError('two independent completed distance measurements required')
    ratios=[]
    for sample in samples:
        raw=float(sample['raw_odom_distance_m']);gt=float(sample['measured_body_gt_x_m'])
        if (sample.get('transport_checks_passed') is not True or
                not math.isfinite(raw+gt) or raw<=0 or gt<=0):
            raise ValueError('invalid completed distance measurement')
        ratios.append(gt/raw)
    scale=float(cert['metres_per_raw_odom_metre'])
    method=cert.get('fit_method','mean')
    if method not in ('mean','min_minus_observed_range'):raise ValueError('unknown distance fit method')
    expected=sum(ratios)/len(ratios) if method=='mean' else min(ratios)-(max(ratios)-min(ratios))
    if not math.isfinite(scale) or scale<=0 or abs(scale-expected)>1e-12:
        raise ValueError('distance scale differs from measured ratios')
    if cert.get('raw_command_cap_m_s')!=.005:raise ValueError('distance calibration speed scope differs')
    return scale


def scaled_odometry(msg, scale):
    """Private leader estimate; leave the ROS message and all topics unchanged."""
    result=copy.deepcopy(msg)
    for axis in ('x','y'):
        setattr(result.pose.pose.position,axis,getattr(msg.pose.pose.position,axis)*scale)
        setattr(result.twist.twist.linear,axis,getattr(msg.twist.twist.linear,axis)*scale)
    # Express the covariance in the same internal metric coordinates.
    factors=(scale,scale,1.,1.,1.,1.)
    for field in ('pose','twist'):
        original=getattr(msg,field).covariance
        getattr(result,field).covariance=[original[i*6+j]*factors[i]*factors[j] for i in range(6) for j in range(6)]
    return result
