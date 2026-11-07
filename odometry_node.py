import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.wheel_radius = 0.065
        self.wheel_separation = 0.36
        
        # Pure relative zero tracking origin
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        self.last_rev_left = None
        self.last_rev_right = None
        self.last_time = self.get_clock().now()
        
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub_joints = self.create_subscription(JointState, '/joint_states', self.encoder_callback, 10)
        self.get_logger().info('Alfrid Clean Kinematics Base Driver Active')

    def encoder_callback(self, msg):
        try:
            if len(msg.position) < 2:
                return
            
            now_time = self.get_clock().now()
            dt = (now_time.nanoseconds - self.last_time.nanoseconds) / 1e9
            
            rev_left = None
            rev_right = None
            for i, name in enumerate(msg.name):
                if 'left' in name.lower():
                    rev_left = float(msg.position[i])
                elif 'right' in name.lower():
                    rev_right = float(msg.position[i])
            
            if rev_left is None or rev_right is None:
                rev_right = float(msg.position[0])
                rev_left = float(msg.position[1])
            
            if self.last_rev_left is None:
                self.last_rev_left = rev_left
                self.last_rev_right = rev_right
                self.last_time = now_time
                return
            
            if dt <= 0.001:
                return
                
            delta_rev_left = rev_left - self.last_rev_left
            delta_rev_right = rev_right - self.last_rev_right
            
            self.last_rev_left = rev_left
            self.last_rev_right = rev_right
            self.last_time = now_time
            
            d_left = delta_rev_left * 2.0 * math.pi * self.wheel_radius
            d_right = delta_rev_right * 2.0 * math.pi * self.wheel_radius
            
            d_center = (d_left + d_right) / 2.0
            d_theta = (d_left - d_right) / self.wheel_separation
            
            self.linear_vel = d_center / dt
            self.angular_vel = d_theta / dt
            
            self.x += d_center * math.cos(self.theta + d_theta / 2.0)
            self.y += d_center * math.sin(self.theta + d_theta / 2.0)
            self.theta += d_theta
            
            self.broadcast_odometry_data(now_time.to_msg())
            
        except Exception as e:
            self.get_logger().error(f'Encoder calculation exception: {e}')

    def broadcast_odometry_data(self, current_stamp):
        odom = Odometry()
        odom.header.stamp = current_stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = float(self.linear_vel)
        odom.twist.twist.angular.z = float(self.angular_vel)
        self.odom_pub.publish(odom)
        
        t_odom = TransformStamped()
        t_odom.header.stamp = current_stamp
        t_odom.header.frame_id = 'odom'
        t_odom.child_frame_id = 'base_footprint'
        t_odom.transform.translation.x = self.x
        t_odom.transform.translation.y = self.y
        t_odom.transform.translation.z = 0.0
        t_odom.transform.rotation.x = 0.0
        t_odom.transform.rotation.y = 0.0
        t_odom.transform.rotation.z = math.sin(self.theta / 2.0)
        t_odom.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t_odom)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
malharlabade@alfrid3:~$ 
