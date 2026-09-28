"""ROS integration check: model selection and exact transform metadata."""
import os,signal,subprocess,time
from pathlib import Path
import rclpy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
rclpy.init();node=rclpy.create_node('pose_filter_integration_test')
received=[];subscription=node.create_subscription(TFMessage,'/test/filtered',received.append,10)
publisher=node.create_publisher(TFMessage,'/test/poses',10)
exe=Path('install/cooperative_transport_gazebo/lib/cooperative_transport_gazebo/transport_pose_filter').resolve()
process=subprocess.Popen([str(exe),'--ros-args','-p','input_topic:=/test/poses','-p','output_topic:=/test/filtered'],stdout=subprocess.DEVNULL)
try:
 deadline=time.monotonic()+10
 while publisher.get_subscription_count()==0 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
 assert publisher.get_subscription_count()>0,'filter did not subscribe'
 message=TFMessage()
 for i,name in enumerate(['unneeded_mesh','cooperative_payload','amir2','amir1']):
  t=TransformStamped();t.child_frame_id=name;t.header.frame_id='world';t.header.stamp.sec=17;t.header.stamp.nanosec=42+i;t.transform.translation.x=float(i);t.transform.rotation.w=1.0;message.transforms.append(t)
 while not received and time.monotonic()<deadline:publisher.publish(message);rclpy.spin_once(node,timeout_sec=.1)
 assert received,'no filtered message'
 output=received[-1];assert [t.child_frame_id for t in output.transforms]==['amir1','amir2','cooperative_payload']
 for t in output.transforms:
  original=next(x for x in message.transforms if x.child_frame_id==t.child_frame_id);assert t==original
 print('PASS: exact positions, quaternions, frame IDs, timestamps; three models only')
finally:
 process.send_signal(signal.SIGINT);process.wait(timeout=5);node.destroy_node();rclpy.shutdown()
