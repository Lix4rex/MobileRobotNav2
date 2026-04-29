#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class OesophageController : public rclcpp::Node {

        public:
                OesophageController() : Node("oesophage_controller"){

                        ask_grab_jengas_topic_sub = this->create_subscription<std_msgs::msg::Bool>(
                                "/start_grab_jengas", 10,
                                std::bind(&OesophageController::grab_jengas_callback, this, std::placeholders::_1)
                        );
                        answer_grab_jengas_topic_pub = this->create_publisher<std_msgs::msg::Bool>(
                                "/grab_jengas_answer", 10
                        );
                        grabbing_jengas = false;
                        
                }

        private: 
                rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ask_grab_jengas_topic_sub;
                rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr answer_grab_jengas_topic_pub;
                bool grabbing_jengas;

                void grab_jengas_callback(const std_msgs::msg::Bool::SharedPtr msg){
                        if (!msg->data || grabbing_jengas) {RCLCPP_INFO(this->get_logger(), "ALREADY GRABBING JENGAS"); return;}
                        grabbing_jengas = true;
                        begin_grabbing_jengas();
                }

                void begin_grabbing_jengas(){
                        RCLCPP_INFO(this->get_logger(), "BEGIN GRABBING JENGAS");
                        grabbing_fake_running_timer_ = this->create_wall_timer(
                                std::chrono::seconds(5),
                                std::bind(&OesophageController::end_grabbing, this)
                        );
                }
                void end_grabbing(){
                        RCLCPP_INFO(this->get_logger(), "JENGAS IN STOMACH");
                        std_msgs::msg::Bool msg;
                        msg.data = true;
                        answer_grab_jengas_topic_pub->publish(msg);
                        grabbing_fake_running_timer_->cancel();
                        grabbing_jengas = false;
                }
                rclcpp::TimerBase::SharedPtr grabbing_fake_running_timer_;
};


int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<OesophageController>());
        rclcpp::shutdown();
        return 0;
}
