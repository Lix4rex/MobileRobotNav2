#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class MatchMonitor : public rclcpp::Node {

        public:
                MatchMonitor() : Node("match_monitor"){
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
                        ask_grab_jengas_topic_pub = this->create_publisher<std_msgs::msg::Bool>(
                                "/start_grab_jengas", 10
                        );

                        answer_grab_jengas_topic_sub = this->create_subscription<std_msgs::msg::Bool>(
                                "/grab_jengas_answer", 10,
                                std::bind(&MatchMonitor::grab_jengas_answer_callback, this, std::placeholders::_1)
                        );

                        // Stomach
                        ask_use_selector_jengas_topic_pub = this->create_publisher<std_msgs::msg::Bool>(
                                "/start_select_jengas", 10
                        );

                        answer_select_jengas_topic_sub = this->create_subscription<std_msgs::msg::Bool>(
                                "/select_jengas_answer", 10,
                                std::bind(&MatchMonitor::select_jengas_answer_callback, this, std::placeholders::_1)
                        );

                        // Anus
                        ask_drop_jengas_topic_pub = this->create_publisher<std_msgs::msg::Bool>(
                                "/start_drop_jengas", 10
                        );

                        answer_drop_jengas_topic_sub = this->create_subscription<std_msgs::msg::Bool>(
                                "/drop_jengas_answer", 10,
                                std::bind(&MatchMonitor::drop_jengas_answer_callback, this, std::placeholders::_1)
                        );


                }

        private: 
                rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_topic_sub;
                double speed;
                void speed_callback(const std_msgs::msg::Float64::SharedPtr msg){
                        speed = msg->data;
                }

                
                rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr launch_match_sub;
                bool match_running;
                void launch_match_callback(const std_msgs::msg::Bool::SharedPtr msg){
                        match_running = msg->data;
                        if (!msg->data)return;
                }


                // Controller actions

                // Oesophage
                rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ask_grab_jengas_topic_pub;
                rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr answer_grab_jengas_topic_sub;
                void grab_jengas_answer_callback(const std_msgs::msg::Bool::SharedPtr msg){
                        if (!msg->data)return;
                        RCLCPP_INFO(this->get_logger(), "SUCCESSFULLY GRABBED JENGAS. WAITING TO BE SELECTED");
                }

                // Stomach
                rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ask_use_selector_jengas_topic_pub;
                rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr answer_select_jengas_topic_sub;
                void select_jengas_answer_callback(const std_msgs::msg::Bool::SharedPtr msg){
                        if (!msg->data)return;
                        RCLCPP_INFO(this->get_logger(), "SUCCESSFULLY STORED JENGAS. WAITING TO BE POOPED");
                }

                // Anus
                rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ask_drop_jengas_topic_pub;
                rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr answer_drop_jengas_topic_sub;
                void drop_jengas_answer_callback(const std_msgs::msg::Bool::SharedPtr msg){
                        if (!msg->data)return;
                        RCLCPP_INFO(this->get_logger(), "SUCCESSFULLY DROPPED JENGAS");
                }
};


int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<MatchMonitor>());
        rclcpp::shutdown();
        return 0;
}
