import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
rclpy.init();node=Node('disturbance_test_guide');stage=0
notices=[(5.,'已站稳，先不要推。'),(10.,'现在请轻推一次，立即松手；只推一次，之后不要再施力。'),(20.,'施力窗口结束；请松开所有按键，观察自行恢复。'),(40.,'正在记录最后 5 秒平衡状态。')]
def state(msg):
 global stage
 t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
 if stage<len(notices) and t>=notices[stage][0]:
  print(f'\a[抗扰提示 {t:.1f}s] {notices[stage][1]}',flush=True);stage+=1
node.create_subscription(JointState,'/joint_states',state,1000)
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:node.destroy_node();rclpy.try_shutdown()
