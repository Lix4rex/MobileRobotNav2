#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

#include <functional>
#include <memory>
#include <thread>

#include "custom_action_interfaces/action/grab_jengas.hpp"
#include "custom_action_interfaces/action/select_jengas.hpp"
#include "custom_action_interfaces/action/drop_jengas.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace car{
class MatchMonitor : public rclcpp::Node {

        
        public:

                using GrabJengas = custom_action_interfaces::action::GrabJengas;
                using GoalHandleGrabJengas = rclcpp_action::ClientGoalHandle<GrabJengas>;

                using SelectJengas = custom_action_interfaces::action::SelectJengas;
                using GoalHandleSelectJengas = rclcpp_action::ClientGoalHandle<SelectJengas>;
                
                using DropJengas = custom_action_interfaces::action::DropJengas;
                using GoalHandleDropJengas = rclcpp_action::ClientGoalHandle<DropJengas>;

                using NavigateToPose = nav2_msgs::action::NavigateToPose;
                using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;


                struct TargetPose {
                        double x;
                        double y;
                        double theta;
                };
                enum class MatchState{
                        IDLE,
                        MOVE_TO_TARGET_POSE,
                        GRAB,
                        SELECT,
                        DROP,
                        FINISHED
                };


                explicit MatchMonitor(const rclcpp::NodeOptions & options) : Node("match_monitor", options){

                        speed_topic_sub = this->create_subscription<std_msgs::msg::Float64>(
                                "/screen_speed", 10,
                                std::bind(&MatchMonitor::speed_callback, this, std::placeholders::_1)
                        );

                        launch_match_sub = this->create_subscription<std_msgs::msg::Bool>(
                                "/launch_match", 10,
                                std::bind(&MatchMonitor::launch_match_callback, this, std::placeholders::_1)
                        );
                        match_running = false;


                        // Controller actions
                        // Oesophage
                        this->oesophage_client_ptr_ = rclcpp_action::create_client<GrabJengas>(
                                this,
                                "grab_jengas"
                        );

                        // Stomach
                        this->stomach_client_ptr_ = rclcpp_action::create_client<SelectJengas>(
                                this,
                                "select_jengas"
                        );

                        // Anus
                        this->anus_client_ptr_ = rclcpp_action::create_client<DropJengas>(
                                this,
                                "drop_jengas"
                        );
                        
                        this->nav_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
                                this,
                                "navigate_to_pose"
                        );
                }


                void send_oesophage_goal(){
                        using namespace std::placeholders;

                        auto goal_msg = GrabJengas::Goal();
                        goal_msg.order = true;

                        RCLCPP_INFO(this->get_logger(), "Sending goal");

                        auto send_goal_options = rclcpp_action::Client<GrabJengas>::SendGoalOptions();
                        send_goal_options.goal_response_callback = [this](const GoalHandleGrabJengas::SharedPtr & goal_handle){
                                if (!goal_handle) {
                                        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                                } else {
                                        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
                                }
                        };

                        send_goal_options.feedback_callback = [this](
                        GoalHandleGrabJengas::SharedPtr,
                        const std::shared_ptr<const GrabJengas::Feedback> feedback)
                        {
                                std::stringstream ss;
                                if (feedback->running){
                                        ss << "RUNNING";
                                } else{
                                        ss << "NOT RUNNING";
                                }
                                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                        };

                        send_goal_options.result_callback = [this](const GoalHandleGrabJengas::WrappedResult & result){
                                switch (result.code) {
                                        case rclcpp_action::ResultCode::SUCCEEDED:
                                        break;
                                        case rclcpp_action::ResultCode::ABORTED:
                                        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                                        return;
                                        case rclcpp_action::ResultCode::CANCELED:
                                        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                                        return;
                                        default:
                                        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                                        return;
                                }
                                std::stringstream ss;
                                ss << "SUCCESSFULLY GRABBED ";
                                ss << result.result->grabbed_nb;
                                ss << " JENGAS";
                                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                                current_state_ = MatchState::SELECT;
                                match_loop();
                        };
                        this->oesophage_client_ptr_->async_send_goal(goal_msg, send_goal_options);
                }


                void send_stomach_goal(){
                        using namespace std::placeholders;

                        auto goal_msg = SelectJengas::Goal();
                        goal_msg.order = true;

                        RCLCPP_INFO(this->get_logger(), "Sending goal");

                        auto send_goal_options = rclcpp_action::Client<SelectJengas>::SendGoalOptions();
                        send_goal_options.goal_response_callback = [this](const GoalHandleSelectJengas::SharedPtr & goal_handle){
                                if (!goal_handle) {
                                        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                                } else {
                                        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
                                }
                        };

                        send_goal_options.feedback_callback = [this](
                        GoalHandleSelectJengas::SharedPtr,
                        const std::shared_ptr<const SelectJengas::Feedback> feedback)
                        {
                                std::stringstream ss;
                                if (feedback->running){
                                        ss << "RUNNING";
                                } else{
                                        ss << "NOT RUNNING";
                                }
                                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                        };

                        send_goal_options.result_callback = [this](const GoalHandleSelectJengas::WrappedResult & result){
                                switch (result.code) {
                                        case rclcpp_action::ResultCode::SUCCEEDED:
                                        break;
                                        case rclcpp_action::ResultCode::ABORTED:
                                        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                                        return;
                                        case rclcpp_action::ResultCode::CANCELED:
                                        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                                        return;
                                        default:
                                        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                                        return;
                                }
                                std::stringstream ss;
                                ss << "SUCCESSFULLY SELECTED JENGAS";
                                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                                current_state_ = MatchState::MOVE_TO_TARGET_POSE;
                                match_loop();
                        };
                        this->stomach_client_ptr_->async_send_goal(goal_msg, send_goal_options);
                }


                void send_anus_goal(){
                        using namespace std::placeholders;

                        auto goal_msg = DropJengas::Goal();
                        goal_msg.order = true;

                        RCLCPP_INFO(this->get_logger(), "Sending goal");

                        auto send_goal_options = rclcpp_action::Client<DropJengas>::SendGoalOptions();
                        send_goal_options.goal_response_callback = [this](const GoalHandleDropJengas::SharedPtr & goal_handle){
                                if (!goal_handle) {
                                        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                                } else {
                                        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
                                }
                        };

                        send_goal_options.feedback_callback = [this](
                        GoalHandleDropJengas::SharedPtr,
                        const std::shared_ptr<const DropJengas::Feedback> feedback)
                        {
                                std::stringstream ss;
                                if (feedback->running){
                                        ss << "RUNNING";
                                } else{
                                        ss << "NOT RUNNING";
                                }
                                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                        };

                        send_goal_options.result_callback = [this](const GoalHandleDropJengas::WrappedResult & result){
                                switch (result.code) {
                                        case rclcpp_action::ResultCode::SUCCEEDED:
                                        break;
                                        case rclcpp_action::ResultCode::ABORTED:
                                        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                                        return;
                                        case rclcpp_action::ResultCode::CANCELED:
                                        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                                        return;
                                        default:
                                        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                                        return;
                                }
                                std::stringstream ss;
                                ss << "SUCCESSFULLY SELECTED JENGAS";
                                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                                current_state_ = MatchState::FINISHED;
                                match_loop();
                        };
                        this->anus_client_ptr_->async_send_goal(goal_msg, send_goal_options);
                }

                void send_nav_goal(TargetPose target){
                        if (!nav_client_ptr_->wait_for_action_server()) {
                                RCLCPP_ERROR(this->get_logger(), "Nav2 not available");
                                return;
                        }

                        auto goal_msg = NavigateToPose::Goal();

                        goal_msg.pose.header.frame_id = "map";
                        goal_msg.pose.header.stamp = this->now();

                        goal_msg.pose.pose.position.x = target.x;
                        goal_msg.pose.pose.position.y = target.y;

                        // orientation simple (theta → quaternion)
                        goal_msg.pose.pose.orientation.z = sin(target.theta / 2.0);
                        goal_msg.pose.pose.orientation.w = cos(target.theta / 2.0);

                        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

                        send_goal_options.result_callback = [this](const GoalHandleNav::WrappedResult & result){
                                if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
                                        RCLCPP_ERROR(this->get_logger(), "Navigation failed");
                                        return;
                                }

                                RCLCPP_INFO(this->get_logger(), "Arrived at target → GRAB");

                                current_state_ = MatchState::GRAB;
                                send_oesophage_goal();
                        };

                        nav_client_ptr_->async_send_goal(goal_msg, send_goal_options);
                }


        private:

                rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_topic_sub;
                double speed;
                void speed_callback(const std_msgs::msg::Float64::SharedPtr msg){
                        speed = msg->data;
                }

                MatchState current_state_ = MatchState::IDLE;
                TargetPose current_target_ = {0.0, 0.0, 0.0};
                
                rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr launch_match_sub;
                bool match_running;
                void launch_match_callback(const std_msgs::msg::Bool::SharedPtr msg){
                        if (!msg->data || match_running)return;
                        match_running = true;
                        RCLCPP_INFO(this->get_logger(), "MATCH STARTED");
                        current_state_ = MatchState::MOVE_TO_TARGET_POSE;

                        match_loop();
                }

                void match_loop(){
                        switch(current_state_){
                                case MatchState::MOVE_TO_TARGET_POSE:
                                        send_nav_goal(current_target_);
                                break;

                                case MatchState::GRAB:
                                        send_oesophage_goal();
                                break;

                                case MatchState::SELECT:
                                        send_stomach_goal();
                                break;

                                case MatchState::DROP:
                                        send_anus_goal();
                                break;

                                case MatchState::FINISHED:
                                        RCLCPP_INFO(this->get_logger(), "MATCH FINISHED");
                                break;

                                default:
                                break;
                        }
                }


                // Controller actions

                // Oesophage
                rclcpp_action::Client<GrabJengas>::SharedPtr oesophage_client_ptr_;

                // Stomach
                rclcpp_action::Client<SelectJengas>::SharedPtr stomach_client_ptr_;

                // Anus
                rclcpp_action::Client<DropJengas>::SharedPtr anus_client_ptr_;

                // Navigation
                rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_ptr_;

};
}


RCLCPP_COMPONENTS_REGISTER_NODE(car::MatchMonitor)