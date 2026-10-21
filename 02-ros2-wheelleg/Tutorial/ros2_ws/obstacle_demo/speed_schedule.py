import os, subprocess, signal
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
rclpy.init();n=Node('obstacle_speed_schedule');pub=n.create_publisher(Twist,'/cmd_vel',10)
phase=0
recorder=None
def state(m):
 global phase, recorder
 t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 if phase==0 and t>=5.0:
  msg=Twist();msg.linear.x=.625;pub.publish(msg);phase=1
  print('开始前进：目标速度 0.625 m/s，仿真时间',t,flush=True)
  record_path=os.environ.get('WHEELLEG_RECORD_PATH')
  if record_path:
   args=['gst-launch-1.0','-e','ximagesrc','use-damage=false','num-buffers=225','!',
         'video/x-raw,framerate=15/1','!','videoconvert','!',
         'textoverlay','text=原速仿真 1.0x | 目标速度 0.625 m/s',
         'font-desc=Noto Sans CJK SC 22','valignment=top','halignment=center','shaded-background=true','!',
         'vp8enc','deadline=1','cpu-used=8','threads=4','!','webmmux','!','filesink','location='+record_path]
   recorder=subprocess.Popen(args,stdout=subprocess.DEVNULL)
   print('开始录制 15 秒原速视频：',record_path,flush=True)

 if phase==1 and t>=14.158:
  pub.publish(Twist());phase=2
  print('发送停车指令，继续检查平衡。仿真时间',t,flush=True)
n.create_subscription(JointState,'/joint_states',state,1000)
try:rclpy.spin(n)
except KeyboardInterrupt:pass
finally:
 if recorder is not None and recorder.poll() is None:
  try:recorder.wait(timeout=2)
  except subprocess.TimeoutExpired:
   recorder.send_signal(signal.SIGINT)
   try:recorder.wait(timeout=3)
   except subprocess.TimeoutExpired:recorder.terminate()
 n.destroy_node();rclpy.try_shutdown()
