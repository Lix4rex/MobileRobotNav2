#include <functional>
#include <memory>
#include <thread>

#include "custom_action_interfaces/action/select_jengas.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace car
{
class StomachActionServer : public rclcpp::Node
{
public:
  using SelectJengas = custom_action_interfaces::action::SelectJengas;
  using GoalHandleSelectJengas = rclcpp_action::ServerGoalHandle<SelectJengas>;

  explicit StomachActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("select_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<SelectJengas>(
      this,
      "select_jengas",
      std::bind(&StomachActionServer::handle_goal, this, _1, _2),
      std::bind(&StomachActionServer::handle_cancel, this, _1),
      std::bind(&StomachActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<SelectJengas>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const SelectJengas::Goal> goal){
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    selecting_done_ = false;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleSelectJengas> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleSelectJengas> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&StomachActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleSelectJengas> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "BEGIN SELECTING JENGAS");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<SelectJengas::Feedback>();

    auto & running = feedback->running;
    running = true;
    auto result = std::make_shared<SelectJengas::Result>();
    begin_selecting_jengas();

    while (rclcpp::ok() && !selecting_done_) {
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");
      
      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->answer = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "SUCCESSFULLY STORED GRABBED JENGAS WITH CORRECT COLOR ");
    }
  }

  void begin_selecting_jengas(){
    selecting_fake_running_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&StomachActionServer::end_selecting, this)
    );
  }
  void end_selecting(){
    selecting_fake_running_timer_->cancel();
    selecting_done_ = true;
  }
  rclcpp::TimerBase::SharedPtr selecting_fake_running_timer_;
  bool selecting_done_;
};

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(car::StomachActionServer)