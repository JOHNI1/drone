#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/apply_link_wrench.hpp>

class ApplyForceNode : public rclcpp::Node
{
public:
  ApplyForceNode()
  : Node("apply_force_node")
  {
    client_ = this->create_client<gazebo_msgs::srv::ApplyLinkWrench>("/apply_link_wrench");
    timer_ = this->create_wall_timer(
      std::chrono::seconds(10), std::bind(&ApplyForceNode::send_request, this));
  }

private:
  void send_request()
  {
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "Service /apply_link_wrench is not available.");
      return;
    }

    auto request = std::make_shared<gazebo_msgs::srv::ApplyLinkWrench::Request>();
    request->link_name = "copterPIX::root_link";  // Ensure this link name matches your model
    request->reference_frame = "";  // Empty reference frame means inertial frame
    request->reference_point.x = 0.0;
    request->reference_point.y = 0.0;
    request->reference_point.z = -0.35;     // set the position of the m16 relative to root_joint
    request->wrench.force.x = -2684.0;      // change with the actual kickback force of the gun!
    request->wrench.force.y = 0.0;      
    request->wrench.force.z = 0.0;
    request->wrench.torque.x = 0.0;
    request->wrench.torque.y = 0.0;
    request->wrench.torque.z = 0.0;
    request->start_time.sec = 0;
    request->start_time.nanosec = 0;
    request->duration.sec = -1;  // Negative duration means apply indefinitely
    request->duration.nanosec = 0;

    auto future_result = client_->async_send_request(request);
    auto status = future_result.wait_for(std::chrono::seconds(2));

    if (status == std::future_status::ready) {
      RCLCPP_INFO(this->get_logger(), "Force applied successfully.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service apply_link_wrench.");
    }
  }

  rclcpp::Client<gazebo_msgs::srv::ApplyLinkWrench>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApplyForceNode>());
  rclcpp::shutdown();
  return 0;
}



