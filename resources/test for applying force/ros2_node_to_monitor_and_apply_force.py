import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench, Vector3
from std_srvs.srv import Empty

class KnobMonitor(Node):

    def __init__(self):
        super().__init__('knob_monitor')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.apply_wrench_client = self.create_client(ApplyBodyWrench, '/apply_body_wrench')
        self.reset_wrench_client = self.create_client(Empty, '/reset_body_wrench')
        self.force_applied = False

    def listener_callback(self, msg):
        knob_position = None
        for i, name in enumerate(msg.name):
            if name == 'knob_joint':
                knob_position = msg.position[i]
                break

        if knob_position is not None:
            if not self.force_applied and abs(knob_position) > 1.22:  # 70 degrees in radians
                self.apply_force()
                self.force_applied = True
            elif abs(knob_position) < 1.05:  # Reset threshold, 60 degrees in radians
                self.force_applied = False

    def apply_force(self):
        req = ApplyBodyWrench.Request()
        req.body_name = 'my_robot::base_link'
        req.wrench.force = Vector3(x=10.0, y=0.0, z=0.0)
        req.duration.sec = 1
        while not self.apply_wrench_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.apply_wrench_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    knob_monitor = KnobMonitor()
    rclpy.spin(knob_monitor)
    knob_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
