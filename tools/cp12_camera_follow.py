#!/usr/bin/env python3
"""Evaluation-only Gazebo camera following; no ROS or robot command interface."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time


def local_offset(world_offset, orientation):
    """CameraTracking offsets use the target frame; convert measured rotation."""
    q = [float(orientation.get(k, 0.)) for k in ('x','y','z','w')]
    norm = sum(v*v for v in q)
    if not all(math.isfinite(v) for v in q) or abs(norm-1.) > .001:
        raise ValueError('finite normalized evaluation target quaternion required')
    # Rotate by the conjugate quaternion, without a robot/control dependency.
    q = [-v/math.sqrt(norm) for v in q[:3]]+[q[3]/math.sqrt(norm)]
    cross = lambda a,b: [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
    t = [2*v for v in cross(q[:3],world_offset)]
    second = cross(q[:3],t)
    return [v+q[3]*a+b for v,a,b in zip(world_offset,t,second)]


def topic_sample(topic):
    command = ['ign','topic','-e','-t',topic,'-n','1','--json-output']
    result = subprocess.run(command,text=True,capture_output=True,timeout=5.)
    if result.returncode != 0:
        raise RuntimeError('GUI evaluation topic unavailable: '+topic)
    return json.loads(result.stdout)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not os.getenv('IGN_PARTITION', '').startswith('amir_cp9_'):
        raise ValueError('isolated simulation partition required')
    # Installed CameraTracking ignores an offset while no target is selected,
    # and interprets offsets in the target's LOCAL frame. A successful service
    # response only acknowledges receipt. Verify the resulting GUI camera pose.
    # Gazebo pose observation here is evaluation-only; this process has no ROS
    # node and no robot command/service interface.
    desired = [0.,-4.2,2.6]
    records = []
    deadline = time.monotonic()+120.
    proof = {'passed':False,'evaluation_only':True,'IGN_PARTITION':os.getenv('IGN_PARTITION'),
        'desired_world_offset':desired,'requests':records,'pose_tolerance_m':.1}
    last_request = -math.inf
    try:
        while time.monotonic() < deadline:
            try:
                scene = topic_sample('/world/cooperative_transport/pose/info')
                target = next(p for p in scene['pose'] if p.get('name')=='cooperative_payload')
                offset = local_offset(desired,target['orientation'])
                if time.monotonic()-last_request >= 2.:
                    requests = [('/gui/follow','ignition.msgs.StringMsg','data: "cooperative_payload"'),
                        ('/gui/follow/offset','ignition.msgs.Vector3d',
                         ' '.join(k+': '+format(v,'.17g') for k,v in zip('xyz',offset)))]
                    for service,reqtype,request in requests:
                        cmd = ['ign','service','-s',service,'--reqtype',reqtype,
                            '--reptype','ignition.msgs.Boolean','--timeout','2000','--req',request]
                        result = subprocess.run(cmd,text=True,capture_output=True,timeout=6.)
                        records.append({'command':cmd,'returncode':result.returncode,'response':result.stdout})
                        if result.returncode != 0 or 'data: true' not in result.stdout:
                            raise RuntimeError('GUI camera request not acknowledged: '+service)
                    last_request = time.monotonic()
                camera = topic_sample('/gui/camera/pose')
                measured = [float(camera['position'].get(k,0.))-float(target['position'].get(k,0.)) for k in 'xyz']
                error = max(abs(a-b) for a,b in zip(measured,desired))
                proof.update({'target_pose':target,'camera_pose':camera,'target_local_offset':offset,
                    'measured_world_offset':measured,'max_offset_error_m':error})
                if all(math.isfinite(v) for v in measured) and error<=proof['pose_tolerance_m']:
                    proof['passed'] = True
                    print('[CP-12 camera] 搬送物を追う側方カメラの実位置を確認しました。',flush=True)
                    return
            except (OSError,ValueError,KeyError,StopIteration,RuntimeError,subprocess.TimeoutExpired) as exc:
                proof['last_retry_reason'] = str(exc)
            time.sleep(.5)
        raise RuntimeError('GUI camera pose did not converge to the requested evaluation view')
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(proof,indent=2))


if __name__ == '__main__':
    main()
