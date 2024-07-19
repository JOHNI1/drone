#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>

class ApplyForceNode : public rclcpp::Node
{
public:
  ApplyForceNode() : Node("apply_force_node")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/apply_force", 10);//This creates a publisher for the Wrench message type on the /apply_force topic with a queue size of 10.
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000), std::bind(&ApplyForceNode::publish_force, this));
  }

private:
  void publish_force()
  {
    auto msg = geometry_msgs::msg::Wrench();
    msg.force.x = 100000.0;
    msg.force.y = 0.0;
    msg.force.z = 0.0;
    msg.torque.x = 0.0;
    msg.torque.y = 0.0;
    msg.torque.z = 0.0;
    publisher_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApplyForceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
