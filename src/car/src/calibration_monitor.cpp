#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class CalibrationMonitor : public rclcpp::Node {

        public:
                CalibrationMonitor() : Node("calibration_monitor"){
                        ask_calibration_topic_sub = this->create_subscription<std_msgs::msg::Bool>(
                                "/start_calibration", 10,
                                std::bind(&CalibrationMonitor::calibration_callback, this, std::placeholders::_1)
                        );

                        answer_calibration_topic_pub = this->create_publisher<std_msgs::msg::Bool>(
                                "/calibration_answer", 10
                        );

                        calibration_running = false;
                }

        private: 

                void calibration_callback(const std_msgs::msg::Bool::SharedPtr msg){
                        if (!msg->data || calibration_running) return;
                        calibration_running = true;
                        begin_calibration();
                }

                void begin_calibration(){
                        RCLCPP_INFO(this->get_logger(), "CALIBRATION BEGAN AND RUNNING");
                        calibration_fake_running_timer_ = this->create_wall_timer(
                                std::chrono::seconds(5),
                                std::bind(&CalibrationMonitor::end_calibration, this)
                        );
                }

                void end_calibration(){
                        RCLCPP_INFO(this->get_logger(), "CALIBRATION DONE");
                        std_msgs::msg::Bool msg;
                        msg.data = true;
                        answer_calibration_topic_pub->publish(msg);
                        calibration_fake_running_timer_->cancel();
                        calibration_running = false;
                }
                rclcpp::TimerBase::SharedPtr calibration_fake_running_timer_;

                rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ask_calibration_topic_sub;
                bool calibration_running;
                rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr answer_calibration_topic_pub;

};


int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<CalibrationMonitor>());
        rclcpp::shutdown();
        return 0;
}
