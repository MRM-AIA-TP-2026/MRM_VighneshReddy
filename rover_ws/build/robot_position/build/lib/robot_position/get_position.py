import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
        
        self.timer = self.create_timer(1.0, self.timer_callback)  
      
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        self.latest_msg = None

    def odom_callback(self, msg):
        
        self.latest_msg = msg

    def timer_callback(self):
        if self.latest_msg is None:
            return  

        position = self.latest_msg.pose.pose.position
        orientation = self.latest_msg.pose.pose.orientation
        self.get_logger().info(f"Position: x={position.x}, y={position.y}, z={position.z}")
        self.get_logger().info(f"Orientation (quaternion): x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")

def main(args=None):
    rclpy.init(args=args)
    odom_listener = OdomListener()
    rclpy.spin(odom_listener)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

