# sudo apt install ros-humble-gazebo-ros-pkgs        is absoltely necessary!!!


import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import ApplyLinkWrench

class ApplyForceNode(Node):

    def __init__(self):
        super().__init__('apply_force_node')
        self.client = self.create_client(ApplyLinkWrench, '/apply_link_wrench')
        self.timer = self.create_timer(5, self.send_request)

    def send_request(self):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service /apply_link_wrench is not available.')
            return

        request = ApplyLinkWrench.Request()
        request.link_name = 'copterPIX::root_link'  # Ensure this link name matches your model
        request.reference_frame = ''  # Empty reference frame means inertial frame
        request.reference_point.x = 0.0
        request.reference_point.y = 0.0
        request.reference_point.z = -0.35  # Set the position relative to root_joint
        request.wrench.force.x = -2684.0  # Change with the actual kickback force of the gun
        request.wrench.force.y = 0.0
        request.wrench.force.z = 0.0
        request.wrench.torque.x = 0.0
        request.wrench.torque.y = 0.0
        request.wrench.torque.z = 0.0
        request.start_time.sec = 0
        request.start_time.nanosec = 0
        request.duration.sec = -1  # Negative duration means apply indefinitely
        request.duration.nanosec = 0

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Force applied successfully.')
            else:
                self.get_logger().error(f'Failed to apply force: {future.result().status_message}')
        else:
            self.get_logger().error('Failed to call service apply_link_wrench.')

def main(args=None):
    rclpy.init(args=args)
    node = ApplyForceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
