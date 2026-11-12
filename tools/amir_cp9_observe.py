#!/usr/bin/env python3
"""Read-only, wall-clock ROS arrival observer. Never sends application commands.

Topic names and exact ROS types must come from an audited graph/configuration.
Arrival timing is not actuator latency, physical sensor accuracy or a safety limit.
"""
import argparse
import json
import math
import statistics
import time
from pathlib import Path


def percentile(values, fraction):
    values = sorted(values)
    if not values:
        return None
    index = (len(values)-1)*fraction
    lo = math.floor(index)
    return values[lo] + (values[math.ceil(index)]-values[lo])*(index-lo)


def finite_payload(value):
    if isinstance(value, dict):
        return all(finite_payload(v) for v in value.values())
    if isinstance(value, (list, tuple)):
        return all(finite_payload(v) for v in value)
    return not isinstance(value, float) or math.isfinite(value)


def message_stamp(data):
    stamp = data.get('header', {}).get('stamp')
    if stamp is None:
        return None
    sec, nano = stamp.get('sec'), stamp.get('nanosec')
    if not isinstance(sec, int) or not isinstance(nano, int) or not 0 <= nano < 10**9:
        return None
    return sec + nano/10**9


def summarize(rows, duration):
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError('duration must be positive and finite')
    intervals = [b['receive_monotonic_s']-a['receive_monotonic_s'] for a,b in zip(rows,rows[1:])]
    stamps = [r.get('source_stamp_s') for r in rows]
    pairs = [(a,b) for a,b in zip(stamps,stamps[1:]) if a is not None and b is not None]
    differences = [b-a for a,b in pairs]
    elapsed = rows[-1]['receive_monotonic_s']-rows[0]['receive_monotonic_s'] if len(rows)>1 else 0
    return {'samples':len(rows),'observed_window_wall_s':duration,
        'arrival_rate_hz':(len(rows)-1)/elapsed if elapsed>0 else None,
        'arrival_gap_median_s':statistics.median(intervals) if intervals else None,
        'arrival_gap_p95_s':percentile(intervals,.95), 'arrival_gap_max_s':max(intervals) if intervals else None,
        'header_samples':sum(s is not None for s in stamps),
        'zero_source_stamps':sum(s==0 for s in stamps),
        'duplicate_source_stamps':sum(d==0 for d in differences),
        'regressing_source_stamps':sum(d<0 for d in differences),
        'source_period_median_s':statistics.median([d for d in differences if d>0]) if any(d>0 for d in differences) else None,
        'nonfinite_payload_samples':sum(not r['finite_payload'] for r in rows),
        'observation_status':'NO_SAMPLES' if not rows else 'OBSERVED_ONLY',
        'sensor_accuracy_verified':False,'end_to_end_actuator_latency_measured':False,
        'firmware_timeout_measured':False,'hardware_qualified':False}


def load_topics(path):
    topics = json.loads(path.read_text())['topics']
    if not topics or len({r['topic'] for r in topics}) != len(topics):
        raise ValueError('explicit nonempty unique topics required')
    for item in topics:
        if not item['topic'].startswith('/') or '/msg/' not in item['type']:
            raise ValueError('absolute topic and explicit message type required')
    return topics


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--topics-file',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--duration',type=float,default=10.)
    a=p.parse_args()
    if not math.isfinite(a.duration) or not 0<a.duration<=3600:
        raise ValueError('bounded finite wall-clock duration required')
    topics=load_topics(a.topics_file)
    a.output.mkdir(parents=True,exist_ok=False)
    import os
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message
    rclpy.init(args=[])
    node=Node('cp9_read_only_observer',enable_rosout=False,start_parameter_services=False)
    rows={r['topic']:[] for r in topics}
    subscriptions=[]
    graph={}
    raw=(a.output/'samples.jsonl').open('w',buffering=1)
    start=time.monotonic()
    def receive(topic,msg):
        value=message_to_ordereddict(msg)
        row={'topic':topic,'receive_monotonic_s':time.monotonic(),'source_stamp_s':message_stamp(value),
            'finite_payload':finite_payload(value),'data':value}
        rows[topic].append({k:v for k,v in row.items() if k!='data'})
        raw.write(json.dumps(row,allow_nan=True)+'\n')
    try:
        for item in topics:
            topic=item['topic']
            subscriptions.append(node.create_subscription(get_message(item['type']),topic,
                lambda msg,t=topic:receive(t,msg),qos_profile_sensor_data))
        while time.monotonic()-start<a.duration:
            rclpy.spin_once(node,timeout_sec=min(.05,max(0,a.duration-(time.monotonic()-start))))
        for item in topics:
            topic=item['topic']
            graph[topic]=[{'node_name':v.node_name,'node_namespace':v.node_namespace,
                'type':v.topic_type,'qos':str(v.qos_profile)} for v in node.get_publishers_info_by_topic(topic)]
        own_publishers=node.get_publisher_names_and_types_by_node(node.get_name(),node.get_namespace())
        report={'date_jst':'2026-10-20','argv':__import__('sys').argv,'ROS_DOMAIN_ID':os.getenv('ROS_DOMAIN_ID'),
            'ROS_LOCALHOST_ONLY':os.getenv('ROS_LOCALHOST_ONLY'), 'clock_basis':'host monotonic receive time',
            'qos':'best effort volatile; acquisition may drop messages',
            'own_publishers':own_publishers,'application_command_publishers':[],
            'hardware_identity':'unverified; topic presence is not hardware identity',
            'hardware_motion_gate_open':False,
            'topics':{t:summarize(rs,time.monotonic()-start) for t,rs in rows.items()}}
        (a.output/'graph.json').write_text(json.dumps(graph,indent=2)+'\n')
        (a.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))
    finally:
        raw.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
