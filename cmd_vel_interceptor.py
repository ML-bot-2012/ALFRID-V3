malharlabade@alfrid3:~$ cat /home/malharlabade/cmd_vel_interceptor.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import time

class AlfridVelocityInterceptor(Node):
    def __init__(self):
        super().__init__('cmd_vel_interceptor')
        self.sub_nav = self.create_subscription(Twist, '/cmd_vel_nav', self.listener_callback, 10)
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            map_qos
        )
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        self.publisher = self.create_publisher(Twist, '/cmd_vel_hw', 10)
        self.last_cmd_time = time.time()
        self.costmap = None
        self.out_of_bounds = False
        self.was_moving = False

        self.timer = self.create_timer(0.05, self.watchdog_callback)
        self.get_logger().info('Alfrid Dual-Topic Direct-Drive Interceptor Active!')

    def costmap_callback(self, msg):
        self.costmap = msg

    def pose_callback(self, msg):
        if self.costmap is None:
            return
        info = self.costmap.info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        max_x = ox + info.width * res
        max_y = oy + info.height * res

        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        margin = 0.20

        if px < (ox + margin) or px > (max_x - margin) or py < (oy + margin) or py > (max_y - margin):
            if not self.out_of_bounds:
                self.get_logger().warn(f'Costmap boundary reached at ({px:.2f}, {py:.2f})! Stopping.')
            self.out_of_bounds = True
            self.stop_motors()
        else:
            self.out_of_bounds = False

    def stop_motors(self):
        for _ in range(5):
            self.publisher.publish(Twist())

    def watchdog_callback(self):
        node_names = self.get_node_names()
        if any('teleop' in name for name in node_names):
            return
        if self.out_of_bounds or (time.time() - self.last_cmd_time > 0.20):
            if self.was_moving:
                self.stop_motors()
                self.was_moving = False
            else:
                self.publisher.publish(Twist())

    def listener_callback(self, msg):
        node_names = self.get_node_names()
        if any('teleop' in name for name in node_names):
            return

        if self.out_of_bounds:
            self.stop_motors()
            return

        lin_in = msg.linear.x
        ang_in = msg.angular.z

        if abs(lin_in) < 0.02 and abs(ang_in) < 0.05:
            if self.was_moving:
                self.stop_motors()
                self.was_moving = False
            else:
                self.publisher.publish(Twist())
            return

        self.last_cmd_time = time.time()
        self.was_moving = True

        out = Twist()
        lin = -lin_in
        ang = ang_in

        if abs(lin) >= 0.02:
            sign_l = 1.0 if lin > 0 else -1.0
            lin = sign_l * min(0.40, 0.20 + (abs(lin) * 0.45))
        else:
            lin = 0.0

        if abs(ang) >= 0.05:
            sign_a = 1.0 if ang > 0 else -1.0
            ang = sign_a * min(0.45, 0.16 + (abs(ang) * 0.35))
        else:
            ang = 0.0

        out.linear.x = lin
        out.angular.z = ang
        self.publisher.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = AlfridVelocityInterceptor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
malharlabade@alfrid3:~$ 
