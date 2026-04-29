#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class ScreenMonitor : public rclcpp::Node {

        public:
                ScreenMonitor() : Node("screen_monitor"){

                        speed_topic_pub = this->create_publisher<std_msgs::msg::Float64>(
                                "/screen_speed", 10
                        );

                        this->declare_parameter<double>("speed", 10.0);
                        speed = this->get_parameter("speed").as_double();

                        parameters_timer_ = this->create_wall_timer(
                                500ms,
                                std::bind(&ScreenMonitor::publish_params, this)
                        );

                        
                        // Calibration action

                        this->declare_parameter<bool>("calibration_pressed", false);
                        calibration_pressed = this->get_parameter("calibration_pressed").as_bool();
                        calibration_done = false;

                        calibration_timer_ = this->create_wall_timer(
                                500ms,
                                std::bind(&ScreenMonitor::begin_calibration, this)
                        );

                        asking_calibration_topic_pub = this->create_publisher<std_msgs::msg::Bool>(
                                "/start_calibration", 10
                        );

                        answer_calibration_topic_sub = this->create_subscription<std_msgs::msg::Bool>(
                                "/calibration_answer", 10,
                                std::bind(&ScreenMonitor::calibration_answer_callback, this, std::placeholders::_1)
                        );

                }

        private: 

                void publish_params(){
                        // speed publication
                        std_msgs::msg::Float64 msg;
                        msg.data = speed;
                        speed_topic_pub->publish(msg);

                        // other parameters publication
                        // ...
                }

                void begin_calibration(){
                        if (!calibration_pressed) return;
                        calibration_done = false;
                        publish_asking_calibration(true);
                }

                void calibration_answer_callback(const std_msgs::msg::Bool::SharedPtr msg){
                        calibration_done = msg->data;
                        calibration_pressed = false;
                        publish_asking_calibration(false);
                        RCLCPP_INFO(this->get_logger(), "CALIBRATION DONE, MATCH CAN BE LAUNCHED");
                }

                void publish_asking_calibration(bool asking){
                        std_msgs::msg::Bool msg;
                        msg.data = asking;
                        asking_calibration_topic_pub->publish(msg);
                }

                rclcpp::TimerBase::SharedPtr parameters_timer_;

                rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_topic_pub;
                double speed;

                
                rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr asking_calibration_topic_pub;
                rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr answer_calibration_topic_sub;
                rclcpp::TimerBase::SharedPtr calibration_timer_;
                bool calibration_pressed;
                bool calibration_done;

};


int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<ScreenMonitor>());
        rclcpp::shutdown();
        return 0;
}
