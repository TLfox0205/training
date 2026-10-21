import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
rclpy.init();n=Node('acceleration_test_schedule');pub=n.create_publisher(Twist,'/cmd_vel',10)
sequence=[(5.0,.2),(6.0,0.),(7.0,-.2),(8.0,0.)];index=0
def state(m):
 global index
 t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 if index<len(sequence) and t>=sequence[index][0]:
  msg=Twist();msg.linear.x=sequence[index][1];pub.publish(msg)
  print(f'仿真 {t:.3f}s：目标速度切换为 {msg.linear.x:+.2f} m/s',flush=True)
  index+=1
n.create_subscription(JointState,'/joint_states',state,1000)
try:rclpy.spin(n)
except KeyboardInterrupt:pass
finally:n.destroy_node();rclpy.try_shutdown()
