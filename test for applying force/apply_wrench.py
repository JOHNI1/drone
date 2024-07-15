import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node

class ForcePublisher(Node):
    def _init_(self):
        super()._init_('force_publisher')
        self.publisher_ = self.create_publisher(WrenchStamped, '/force_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_message)

    def publish_message(self):
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.wrench.force.x = 1.0  # Apply force in x-direction
        msg.wrench.force.y = 0.0
        msg.wrench.force.z = 0.0
        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    force_publisher = ForcePublisher()
    rclpy.spin(force_publisher)

if __name__ == '_main_':
    main()

