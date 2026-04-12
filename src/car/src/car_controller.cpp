#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;

class CarController : public rclcpp::Node {

        public:
                CarController() : Node("car_controller"){

                        joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
                                "/joy", 10,
                                std::bind(&CarController::joy_callback, this, std::placeholders::_1)
                        );

                        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(
                                "/diff_drive_controller/cmd_vel", 10
                        );

                        cmd_vel_nav2_sub = this->create_subscription<geometry_msgs::msg::Twist>(
                                "/cmd_vel", 10, 
                                std::bind(&CarController::cmd_vel_nav2_callback, this, std::placeholders::_1)
                        );

                        this->declare_parameter<bool>("controlable");
                        controlable = this->get_parameter("controlable").as_bool();

                        this->declare_parameter<double>("joy_speed");
                        joy_speed = this->get_parameter("joy_speed").as_double();

                        this->declare_parameter<double>("nav2_speed");
                        nav2_speed = this->get_parameter("nav2_speed").as_double();

                }

        private: 
                void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy) {

                        auto msg = geometry_msgs::msg::TwistStamped();

                        msg.header.stamp = this->now();
                        msg.header.frame_id = "base_footprint";

                        msg.twist.linear.x = joy->axes[1] * joy_speed;
                        msg.twist.angular.z = joy->axes[3] * joy_speed/3;

                        if (controlable) {
                                cmd_vel_pub->publish(msg);
                        }
                }

                void cmd_vel_nav2_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
                        auto repub_msg = geometry_msgs::msg::TwistStamped();

                        repub_msg.header.stamp = this->now();
                        repub_msg.header.frame_id = "base_footprint";

                        repub_msg.twist.linear.x = nav2_speed*msg->linear.x;
                        repub_msg.twist.angular.z = nav2_speed*msg->angular.z;
                        
                        if (!controlable){
                                cmd_vel_pub->publish(repub_msg);
                        }

                }

                rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
                rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub;
                rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_nav2_sub;

                bool controlable;
                double joy_speed;
                double nav2_speed;
};


int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<CarController>());
        rclcpp::shutdown();
        return 0;
}
