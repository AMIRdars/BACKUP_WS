"""Isolated Gazebo transport -> ROS test, never inject into a live simulation."""
import os, signal, subprocess, time, math
from pathlib import Path
import rclpy
from std_msgs.msg import Bool, Float64MultiArray
assert os.environ.get('IGN_PARTITION', '').startswith('contact-native-test-')
rclpy.init(); node=rclpy.create_node('native_contact_integration_test')
received=[]; metrics=[]
sub=node.create_subscription(Bool,'/amir1/finger_left_contact_detected',lambda m:received.append(m.data),10)
metric_sub=node.create_subscription(Float64MultiArray,'/cooperative_transport/contact_metrics',lambda m:metrics.append(list(m.data)),10)
exe=Path('install/cooperative_transport_gazebo/lib/cooperative_transport_gazebo/finger_contact_aggregator').resolve()
process=subprocess.Popen([str(exe),'--ros-args','-p','input_transport:=gazebo','-p','collision_count:=3','-p','contact_timeout:=2.0'],stdout=subprocess.DEVNULL)
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
def send(index,text):
 suffix='' if index==0 else f'_{index}'
 topic=f'/world/cooperative_transport_friction/model/amir1/link/finger_left_1/sensor/finger_left_contact_sensor{suffix}/contact'
 result=subprocess.run(['ign','topic','-t',topic,'-m','ignition.msgs.Contacts','-p',text],capture_output=True,text=True,timeout=5)
 assert result.returncode==0,result.stderr
 spin(.12)
positive='contact {collision1 {name: "amir1::finger_left_1::collision"} collision2 {name: "cooperative_payload::link::collision"}}'
try:
 spin(1.5)
 assert received,'no ROS output'
 send(0,positive);assert received[-1] and math.isnan(metrics[-1][1]),'positive/missing depth'
 send(1,'');assert received[-1],'empty segment erased positive segment'
 send(0,'');assert not received[-1],'explicit release'
 send(0,positive);assert received[-1]
 spin(2.3);assert not received[-1],'stale expiry'
 measured=positive[:-1]+' depth: 0.001 depth: 0.002}'
 send(0,measured);assert received[-1] and abs(metrics[-1][1]-.002)<1e-10,'depth preservation'
 send(0,positive.replace('cooperative_payload::link::collision','ground_plane'));assert not received[-1],'non-payload exclusion'
 # Other fingers and the reversed collision order must use the same matching semantics.
 reversed_contact='contact {collision2 {name: "amir1::finger_left_1::collision"} collision1 {name: "cooperative_payload::link::collision"}}'
 send(2,reversed_contact);assert received[-1],'reversed order / segment suffix'
 print('PASS: Gazebo native input, OR, explicit release, expiry, payload filter, depth, reverse order')
finally:
 process.send_signal(signal.SIGINT);process.wait(timeout=5)
 node.destroy_node();rclpy.shutdown()
