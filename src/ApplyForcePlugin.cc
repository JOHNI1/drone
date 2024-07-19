#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class ApplyForcePlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      this->model = _parent;

      RCLCPP_INFO(rclcpp::get_logger("apply_force_node"), "ApplyForcePlugin loaded!");

      // Read the link name from the SDF file
      if (_sdf->HasElement("link_name"))
      {
        this->linkName = _sdf->Get<std::string>("link_name");
        RCLCPP_INFO(rclcpp::get_logger("apply_force_node"), "Link name: %s", this->linkName.c_str());
      }
      else
      {
        RCLCPP_ERROR(rclcpp::get_logger("apply_force_node"), "ApplyForcePlugin missing <link_name> parameter.");
        return;
      }

      this->link = this->model->GetLink(this->linkName);
      if (!this->link)
      {
        RCLCPP_ERROR(rclcpp::get_logger("apply_force_node"), "Link named %s not found.", this->linkName.c_str());
        return;
      }

      if (!rclcpp::ok())
      {
        rclcpp::init(0, nullptr);
      }

      this->rosNode = rclcpp::Node::make_shared("apply_force_node");
      this->rosSub = this->rosNode->create_subscription<geometry_msgs::msg::Wrench>(
          "/apply_force", 10, std::bind(&ApplyForcePlugin::OnRosMsg, this, std::placeholders::_1));
      RCLCPP_INFO(rclcpp::get_logger("apply_force_node"), "ROS subscription created, ready to receive force commands.");
    }

  private:
    void OnRosMsg(const geometry_msgs::msg::Wrench::SharedPtr msg)
    {
      RCLCPP_INFO(rclcpp::get_logger("apply_force_node"), "Applying force: [%f, %f, %f]", msg->force.x, msg->force.y, msg->force.z);
      this->link->AddForce(ignition::math::Vector3d(msg->force.x, msg->force.y, msg->force.z));
      this->link->AddTorque(ignition::math::Vector3d(msg->torque.x, msg->torque.y, msg->torque.z));
    }

    physics::ModelPtr model;
    physics::LinkPtr link;
    std::string linkName;
    rclcpp::Node::SharedPtr rosNode;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr rosSub;
  };

  GZ_REGISTER_MODEL_PLUGIN(ApplyForcePlugin)
}

















// #include <gazebo/gazebo.hh>
// #include <gazebo/physics/physics.hh>
// #include <gazebo/common/Plugin.hh>
// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/wrench.hpp>
// #include <ignition/math/Vector3.hh>


// namespace gazebo
// {
//   class ApplyForcePlugin : public ModelPlugin
//   {
//   public:
//     void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
//     {
//       this->model = _parent;

//       // Read the link name from the SDF file
//       if (_sdf->HasElement("link_name"))
//       {
//         this->linkName = _sdf->Get<std::string>("link_name");
//       }
//       else
//       {
//         RCLCPP_ERROR(rclcpp::get_logger("apply_force_node"), "ApplyForcePlugin missing <link_name> parameter.");
//         return;
//       }

//       this->link = this->model->GetLink(this->linkName);
//       if (!this->link)
//       {
//         RCLCPP_ERROR(rclcpp::get_logger("apply_force_node"), "Link named %s not found.", this->linkName.c_str());
//         return;
//       }

//       if (!rclcpp::ok())
//       {
//         rclcpp::init(0, nullptr);
//       }

//       this->rosNode = rclcpp::Node::make_shared("apply_force_node");
//       this->rosSub = this->rosNode->create_subscription<geometry_msgs::msg::Wrench>(
//           "/apply_force", 10, std::bind(&ApplyForcePlugin::OnRosMsg, this, std::placeholders::_1));
//     }

//   private:
//     void OnRosMsg(const geometry_msgs::msg::Wrench::SharedPtr msg)
//     {
//       this->link->AddForce(ignition::math::Vector3d(msg->force.x, msg->force.y, msg->force.z));
//       this->link->AddTorque(ignition::math::Vector3d(msg->torque.x, msg->torque.y, msg->torque.z));
//     }

//     physics::ModelPtr model;
//     physics::LinkPtr link;
//     std::string linkName;
//     rclcpp::Node::SharedPtr rosNode;
//     rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr rosSub;
//   };

//   GZ_REGISTER_MODEL_PLUGIN(ApplyForcePlugin)
// }
