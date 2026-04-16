#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>

class GTNode : public rclcpp::Node
{
public:
  GTNode() : Node("gt_node", rclcpp::NodeOptions()
  .parameter_overrides({rclcpp::Parameter("use_sim_time", true)}))
  {
    pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/ground_truth/odom", 10);

    node_.Subscribe(
      "/world/simple_world/pose/info",
      &GTNode::cb,
      this);
  }

private:

  void cb(const gz::msgs::Pose_V & msg)
  {
    for (int i = 0; i < msg.pose_size(); i++)
    {
      const auto & p = msg.pose(i);

      if (p.name() == "simple_robot")
      {
        nav_msgs::msg::Odometry odom;

        odom.header.stamp = this->get_clock()->now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        odom.pose.pose.position.x = p.position().x();
        odom.pose.pose.position.y = p.position().y();
        odom.pose.pose.position.z = p.position().z();

        odom.pose.pose.orientation.x = p.orientation().x();
        odom.pose.pose.orientation.y = p.orientation().y();
        odom.pose.pose.orientation.z = p.orientation().z();
        odom.pose.pose.orientation.w = p.orientation().w();

        pub_->publish(odom);
      }
    }
  }

  gz::transport::Node node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GTNode>());
  rclcpp::shutdown();
}