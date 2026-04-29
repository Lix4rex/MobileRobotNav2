#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class StomachController : public rclcpp::Node {

        public:
                StomachController() : Node("stomach_controller"){
                        ask_select_jengas_topic_sub = this->create_subscription<std_msgs::msg::Bool>(
                                "/start_select_jengas", 10,
                                std::bind(&StomachController::select_jengas_callback, this, std::placeholders::_1)
                        );
                        answer_select_jengas_topic_pub = this->create_publisher<std_msgs::msg::Bool>(
                                "/select_jengas_answer", 10
                        );
                }

        private: 
                rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ask_select_jengas_topic_sub;
                rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr answer_select_jengas_topic_pub;
                bool selecting_jengas;

                void select_jengas_callback(const std_msgs::msg::Bool::SharedPtr msg){
                        if (!msg->data || selecting_jengas){RCLCPP_INFO(this->get_logger(), "ALREADY SELECTING JENGAS"); return;}
                        selecting_jengas = true;
                        begin_selecting_jengas();
                }

                void begin_selecting_jengas(){
                        RCLCPP_INFO(this->get_logger(), "BEGIN SELECTING JENGAS");
                        selecting_fake_running_timer_ = this->create_wall_timer(
                                std::chrono::seconds(5),
                                std::bind(&StomachController::end_selecting, this)
                        );
                }
                void end_selecting(){
                        RCLCPP_INFO(this->get_logger(), "SUCCESSFULLY STORED GRABBED JENGAS WITH CORRECT COLOR");
                        std_msgs::msg::Bool msg;
                        msg.data = true;
                        answer_select_jengas_topic_pub->publish(msg);
                        selecting_fake_running_timer_->cancel();
                        selecting_jengas = false;
                }
                rclcpp::TimerBase::SharedPtr selecting_fake_running_timer_;
};


int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<StomachController>());
        rclcpp::shutdown();
        return 0;
}
