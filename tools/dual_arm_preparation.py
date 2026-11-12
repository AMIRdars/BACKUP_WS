"""Experiment-only physical base approach; retires before dual arm control."""
import json
import math
from geometry_msgs.msg import Twist


def physical_approach(*,node,robots,generated,output,poses,now,phase,wait,sim_wait,report):
    staging=json.loads((generated/'scene_staging.json').read_text())['robots']
    phase[0]='BASE_APPROACH'
    publishers={}
    for robot in robots:
        if node.get_publishers_info_by_topic('/'+robot+'/rover_twist'):
            raise RuntimeError('approach requires exclusive rover_twist writer: '+robot)
        publishers[robot]=node.create_publisher(Twist,'/'+robot+'/rover_twist',1)
    started=now[0];last=[started];velocities={r:0. for r in robots};stable=[None];samples=[]
    previous={r:poses[r].translation.x for r in robots};peak={r:0. for r in robots}
    def step():
        dt=now[0]-last[0]
        if dt<=0:return False
        if dt>.1:raise RuntimeError('physical approach dt abnormal')
        all_stopped=True
        for robot in robots:
            x=poses[robot].translation.x;q=poses[robot].rotation
            yaw=2*math.atan2(q.z,q.w)
            error=staging[robot]['staged_x_m']-x
            desired=max(-.015,min(.015,error*math.cos(yaw)))
            if abs(error)<.00015:desired=0.
            bound=.005*dt
            velocities[robot]=max(velocities[robot]-bound,min(velocities[robot]+bound,desired))
            message=Twist();message.linear.x=velocities[robot];publishers[robot].publish(message)
            measured_speed=abs(x-previous[robot])/dt;previous[robot]=x
            peak[robot]=max(peak[robot],abs(velocities[robot]))
            all_stopped &= abs(error)<.0002 and abs(velocities[robot])<1e-12 and measured_speed<.0001
            samples.append({'state':phase[0],'sim_time':now[0],'robot':robot,'world_x_m':x,
                            'target_x_m':staging[robot]['staged_x_m'],'cmd_vx':velocities[robot],
                            'measured_speed_m_s':measured_speed})
        last[0]=now[0]
        stable[0]=now[0] if all_stopped and stable[0] is None else stable[0] if all_stopped else None
        return stable[0] is not None and now[0]-stable[0]>=.5
    try:
        wait(step,120.)
        phase[0]='BASE_STOP'
        for publisher in publishers.values():publisher.publish(Twist())
        sim_wait(.5,check_fault=False)
        report['physical_base_approach']={'passed':True,'sim_start_s':started,'sim_stopped_s':now[0],
            'stop_before_grasp':True,'peak_command_m_s':peak,
            'final_world_x_m':{r:poses[r].translation.x for r in robots},
            'position_input_scope':'own base world pose during preparation only; publishers retired before ARM_ONLY',
            'pose_service_calls':0,'sample_count':len(samples)}
    finally:
        for publisher in publishers.values():publisher.publish(Twist())
        sim_wait(.1,check_fault=False)
        for publisher in publishers.values():node.destroy_publisher(publisher)
        (output/'physical_approach.json').write_text(json.dumps({'result':report.get('physical_base_approach'),
                                                               'samples':samples},indent=2))
