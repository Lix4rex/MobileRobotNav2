#include <functional>
#include <memory>
#include <thread>

#include "custom_action_interfaces/action/grab_jengas.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace car
{
class OesophageActionServer : public rclcpp::Node
{
public:
  using GrabJengas = custom_action_interfaces::action::GrabJengas;
  using GoalHandleGrabJengas = rclcpp_action::ServerGoalHandle<GrabJengas>;

  explicit OesophageActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("oesophage_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<GrabJengas>(
      this,
      "grab_jengas",
      std::bind(&OesophageActionServer::handle_goal, this, _1, _2),
      std::bind(&OesophageActionServer::handle_cancel, this, _1),
      std::bind(&OesophageActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<GrabJengas>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GrabJengas::Goal> goal){
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    grabbing_done_ = false;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGrabJengas> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGrabJengas> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&OesophageActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGrabJengas> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "BEGIN GRABBING JENGAS");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GrabJengas::Feedback>();

    auto & running = feedback->running;
    running = true;
    auto result = std::make_shared<GrabJengas::Result>();
    begin_grabbing_jengas();

    while (rclcpp::ok() && !grabbing_done_) {
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");
      
      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->grabbed_nb = 4;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "JENGAS IN STOMACH");
    }
  }

  void begin_grabbing_jengas(){
    grabbing_fake_running_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&OesophageActionServer::end_grabbing, this)
    );
  }
  void end_grabbing(){
    grabbing_fake_running_timer_->cancel();
    grabbing_done_ = true;
  }
  rclcpp::TimerBase::SharedPtr grabbing_fake_running_timer_;
  bool grabbing_done_;
};

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(car::OesophageActionServer)