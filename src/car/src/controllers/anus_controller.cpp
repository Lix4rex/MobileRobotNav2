#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class AnusController : public rclcpp::Node {

        public:
                AnusController() : Node("anus_controller"){
                        ask_drop_jengas_topic_sub = this->create_subscription<std_msgs::msg::Bool>(
                                "/start_drop_jengas", 10,
                                std::bind(&AnusController::drop_jengas_callback, this, std::placeholders::_1)
                        );
                        answer_drop_jengas_topic_pub = this->create_publisher<std_msgs::msg::Bool>(
                                "/drop_jengas_answer", 10
                        );
                        
                }

        private: 
                rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ask_drop_jengas_topic_sub;
                rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr answer_drop_jengas_topic_pub;

                bool dropping_jengas;

                void drop_jengas_callback(const std_msgs::msg::Bool::SharedPtr msg){
                        if (!msg->data || dropping_jengas) {RCLCPP_INFO(this->get_logger(), "ALREADY DROPPING JENGAS"); return;}
                        dropping_jengas = true;
                        drop_jengas();
                }

                void drop_jengas(){
                        RCLCPP_INFO(this->get_logger(), "BEGIN DROPPING JENGAS");
                        drop_fake_running_timer_ = this->create_wall_timer(
                                std::chrono::seconds(5),
                                std::bind(&AnusController::end_dropping, this)
                        );
                }
                void end_dropping(){
                        RCLCPP_INFO(this->get_logger(), "SUCCESSFULLY DROPPED JENGAS ON THE FLOOR");
                        std_msgs::msg::Bool msg;
                        msg.data = true;
                        answer_drop_jengas_topic_pub->publish(msg);
                        drop_fake_running_timer_->cancel();
                        dropping_jengas = false;
                }
                rclcpp::TimerBase::SharedPtr drop_fake_running_timer_;
};


int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<AnusController>());
        rclcpp::shutdown();
        return 0;
}
