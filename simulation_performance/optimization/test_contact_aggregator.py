"""ROS integration: any active segment, explicit release, expiry, payload filter."""
import signal,subprocess,time,math
from pathlib import Path
import rclpy
from std_msgs.msg import Bool,Float64MultiArray
from ros_gz_interfaces.msg import Contacts,Contact
rclpy.init();node=rclpy.create_node('contact_aggregator_integration_test')
received=[];sub=node.create_subscription(Bool,'/amir1/finger_left_contact_detected',lambda m:received.append((time.monotonic(),m.data)),10)
metrics=[];metric_sub=node.create_subscription(Float64MultiArray,'/cooperative_transport/contact_metrics',lambda m:metrics.append(list(m.data)),10)
pubs=[node.create_publisher(Contacts,f'/amir1/finger_left_contact_segments/segment_{i}',10) for i in [0,1]]
exe=Path('install/cooperative_transport_gazebo/lib/cooperative_transport_gazebo/finger_contact_aggregator').resolve()
process=subprocess.Popen([str(exe),'--ros-args','-p','collision_count:=3','-p','contact_timeout:=0.3'],stdout=subprocess.DEVNULL)
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
try:
 end=time.monotonic()+10
 while any(p.get_subscription_count()==0 for p in pubs) and time.monotonic()<end:spin(.05)
 assert all(p.get_subscription_count()>0 for p in pubs)
 touching=Contacts();contact=Contact();contact.collision1.name='amir1::finger_left_1::collision';contact.collision2.name='cooperative_payload::link::collision';touching.contacts=[contact]
 pubs[0].publish(touching);pubs[1].publish(Contacts());spin(.15)
 assert any(value for _,value in received),'empty segment erased a positive contact'
 assert metrics and math.isnan(metrics[-1][1]),'missing depth must remain unknown'
 received.clear();pubs[0].publish(Contacts());spin(.1);assert received and received[-1][1] is False,'release not detected'
 pubs[0].publish(touching);spin(.15);assert received[-1][1] is True
 spin(.4);assert received[-1][1] is False,'stale contact did not expire'
 touching.contacts[0].depths=[.001,.002];pubs[0].publish(touching);spin(.1)
 assert metrics[-1][1]==.002,'available depth not preserved'
 unrelated=Contacts();contact=Contact();contact.collision1.name='amir1::finger_left_1::collision';contact.collision2.name='ground_plane';unrelated.contacts=[contact];pubs[0].publish(unrelated);spin(.15);assert received[-1][1] is False
 print('PASS: segment OR, release, stale expiry, non-payload exclusion, missing/measured depth')
finally:
 process.send_signal(signal.SIGINT);process.wait(timeout=5);node.destroy_node();rclpy.shutdown()
