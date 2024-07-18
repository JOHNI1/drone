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






import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, Vector3
from gazebo_msgs.srv import ApplyBodyWrench
from rclpy.qos import QoSProfile

class ApplyForceNode(Node):
    def __init__(self):
        super().__init__('apply_force_node')
        self.publisher_ = self.create_publisher(Wrench, '/apply_force', QoSProfile(depth=10))
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Apply Force Node has been started')

    def timer_callback(self):
        # Create a Wrench message
        wrench_msg = Wrench()
        wrench_msg.force = Vector3(x=10.0, y=0.0, z=0.0)
        wrench_msg.torque = Vector3(x=0.0, y=0.0, z=0.0)

        # Publish the message
        self.publisher_.publish(wrench_msg)
        self.get_logger().info('Applying force: %s' % wrench_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ApplyForceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()






import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Point, Wrench, Vector3

class ApplyForceNode(Node):
    def __init__(self):
        super().__init__('apply_force_node')
        self.cli = self.create_client(ApplyBodyWrench, '/gazebo/apply_body_wrench')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = ApplyBodyWrench.Request()

        # Set the parameters for the request
        self.req.body_name = 'robot::link'  # Replace 'robot::link' with your model's link name
        self.req.reference_frame = 'world'  # Reference frame, 'world' for global coordinates
        self.req.reference_point = Point(x=0.0, y=0.0, z=0.0)  # Point of application of force

        # Wrench: combination of force and torque
        self.req.wrench = Wrench()
        self.req.wrench.force = Vector3(x=10.0, y=0.0, z=0.0)  # Force vector
        self.req.wrench.torque = Vector3(x=0.0, y=0.0, z=0.0)  # Torque vector

        self.req.start_time.sec = 0  # Start time in seconds
        self.req.start_time.nanosec = 0  # Start time in nanoseconds
        self.req.duration.sec = 1  # Duration in seconds
        self.req.duration.nanosec = 0  # Duration in nanoseconds

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.future = self.cli.call_async(self.req)
        self.get_logger().info('Applying force to the model')

def main(args=None):
    rclpy.init(args=args)
    node = ApplyForceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from gazebo_msgs.srv import ApplyBodyWrench

class ApplyForceNode(Node):
    def __init__(self):
        super().__init__('apply_force_node')

        # Create a client for the ApplyBodyWrench service
        self.client = self.create_client(ApplyBodyWrench, '/gazebo/apply_body_wrench')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Create a request
        request = ApplyBodyWrench.Request()
        request.body_name = 'your_model::your_link'
        request.reference_frame = 'world'
        request.wrench.force = Vector3(x=10.0, y=0.0, z=0.0)
        request.wrench.torque = Vector3(x=0.0, y=0.0, z=0.0)
        request.duration.sec = 1
        request.duration.nanosec = 0

        # Call the service
        self.client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    node = ApplyForceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
