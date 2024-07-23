#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/apply_link_wrench.hpp>
#include <thread>

class ApplyForceNode : public rclcpp::Node
{
public:
  ApplyForceNode()
  : Node("apply_force_node")
  {
    client_ = this->create_client<gazebo_msgs::srv::ApplyLinkWrench>("/apply_link_wrench");
    timer_ = this->create_wall_timer(
      std::chrono::seconds(10), std::bind(&ApplyForceNode::apply_force, this));
  }

private:

  void apply_force()
  {
    send_request(-3000.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    send_request(0.0);
  }
  void send_request(float force)
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
    request->wrench.force.x = force;      // change with the actual kickback force of the gun!
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
    // auto status = future_result.wait_for(std::chrono::seconds(2));

    // if (status == std::future_status::ready) {
    //   RCLCPP_INFO(this->get_logger(), "Force applied successfully.");
    // } else {
    //   RCLCPP_ERROR(this->get_logger(), "Failed to call service apply_link_wrench.");
    // }
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



// ~/drone_ws$ ros2 interface show gazebo_msgs/srv/ApplyLinkWrench
// # Apply Wrench to Gazebo Link.
// # via the callback mechanism
// # all Gazebo operations are made in world frame
// string link_name                          # Gazebo link to apply wrench (linear force and torque)
//                                           # wrench is applied in the gazebo world by default
//                                           # link names are prefixed by model name, e.g. pr2::base_link
// string reference_frame                    # wrench is defined in the reference frame of this entity
//                                           # use inertial frame if left empty
//                                           # frame names are links prefixed by model name, e.g. pr2::base_link
// geometry_msgs/Point  reference_point      # wrench is defined at this location in the reference frame
//         float64 x
//         float64 y
//         float64 z
// geometry_msgs/Wrench wrench               # wrench applied to the origin of the link
//         Vector3  force
//                 float64 x
//                 float64 y
//                 float64 z
//         Vector3  torque
//                 float64 x
//                 float64 y
//                 float64 z
// builtin_interfaces/Time start_time        # (optional) wrench application start time (seconds)
//         int32 sec
//         uint32 nanosec
//                                           # if start_time is not specified, or
//                                           # start_time < current time, start as soon as possible
// builtin_interfaces/Duration duration      # optional duration of wrench application time (seconds)
//         int32 sec
//         uint32 nanosec
//                                           # if duration < 0, apply wrench continuously without end
//                                           # if duration = 0, do nothing
//                                           # if duration < step size, apply wrench
//                                           # for one step size
// ---
// bool success                              # return true if set wrench successful
// string status_message                     # comments if available