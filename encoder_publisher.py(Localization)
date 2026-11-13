import os
os.environ['ROS_DOMAIN_ID'] = '0'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import serial
import threading
import time
class EncoderPublisher(Node):
    def __init__(self):
        super().__init__('encoder_publisher')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        try:
            # Open with 0 timeout for true asynchronous non-blocking operation
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
            self.get_logger().info('Serial port /dev/ttyACM0 opened in true non-blocking stream mode')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial connection: {e}')
            return
            
        time.sleep(2)
        self.buffer = ""
        self.get_logger().info('Encoder non-blocking streaming thread active')
        
        self.thread = threading.Thread(target=self.read_serial, daemon=True)
        self.thread.start()
    def cmd_vel_callback(self, msg):
        try:
            cmd_string = f"V:{msg.linear.x:.2f},{msg.angular.z:.2f}\n"
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.write(cmd_string.encode())
        except Exception as e:
            self.get_logger().error(f"Failed writing down serial: {e}")
    def read_serial(self):
        while rclpy.ok():
            if hasattr(self, 'ser') and self.ser.is_open:
                try:
                    # Check if raw bytes are waiting in the Linux kernel UART buffer
                    if self.ser.in_waiting > 0:
                        raw_data = self.ser.read(self.ser.in_waiting)
                        self.buffer += raw_data.decode('utf-8', errors='ignore')
                        
                        # Process individual full lines matching newline breaks
                        while '\n' in self.buffer:
                            # Split the first full line out from the accumulator
                            line, self.buffer = self.buffer.split('\n', 1)
                            line = line.strip()
                            
                            if "A:" in line and "B:" in line and "|" in line:
                                # LOCK TIMESTAMP: Mint a timestamp ONLY when a complete string finishes physical transmission
                                now_stamp = self.get_clock().now().to_msg()
                                
                                parts = line.split('|')
                                if len(parts) >= 2:
                                    str_a = parts[0].replace('A:', '').replace('rev', '').strip()
                                    str_b = parts[1].replace('B:', '').replace('rev', '').strip()
                                    
                                    try:
                                        rev_a = float(str_a)
                                        rev_b = float(str_b)
                                        
                                        msg = JointState()
                                        msg.header.stamp = now_stamp
                                        msg.name = ['right_wheel', 'left_wheel']
                                        msg.position = [float(rev_b), float(rev_a)]
                                        
                                        self.pub.publish(msg)
                                    except ValueError:
                                        pass
                except Exception as e:
                    self.get_logger().error(f"Serial worker read crash: {e}")
            # High-frequency micro-sleep to prevent CPU pinning while maintaining 50Hz capture rates
            time.sleep(0.005) 
def main(args=None):
    rclpy.init(args=args)
    node = EncoderPublisher()
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
