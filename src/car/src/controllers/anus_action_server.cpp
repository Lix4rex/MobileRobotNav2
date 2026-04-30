#include <functional>
#include <memory>
#include <thread>

#include "custom_action_interfaces/action/drop_jengas.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace car
{
class AnusActionServer : public rclcpp::Node
{
public:
  using DropJengas = custom_action_interfaces::action::DropJengas;
  using GoalHandleDropJengas = rclcpp_action::ServerGoalHandle<DropJengas>;

  explicit AnusActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("anus_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<DropJengas>(
      this,
      "drop_jengas",
      std::bind(&AnusActionServer::handle_goal, this, _1, _2),
      std::bind(&AnusActionServer::handle_cancel, this, _1),
      std::bind(&AnusActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<DropJengas>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DropJengas::Goal> goal){
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    dropping_done_ = false;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDropJengas> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleDropJengas> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&AnusActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleDropJengas> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "BEGIN DROPPING JENGAS");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<DropJengas::Feedback>();

    auto & running = feedback->running;
    running = true;
    auto result = std::make_shared<DropJengas::Result>();
    begin_dropping_jengas();

    while (rclcpp::ok() && !dropping_done_) {
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");
      
      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->answer = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "JENGAS DROPPED ON THE FLOOR");
    }
  }

  void begin_dropping_jengas(){
    dropping_fake_running_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&AnusActionServer::end_dropping, this)
    );
  }
  void end_dropping(){
    dropping_fake_running_timer_->cancel();
    dropping_done_ = true;
  }
  rclcpp::TimerBase::SharedPtr dropping_fake_running_timer_;
  bool dropping_done_;
};

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(car::AnusActionServer)