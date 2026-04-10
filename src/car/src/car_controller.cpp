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

                        this->declare_parameter<bool>("controlable");
                        controlable = this->get_parameter("controlable").as_bool();

                        this->declare_parameter<double>("speed");
                        speed = this->get_parameter("speed").as_double();

                }

        private: 
                void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy) {

                        auto msg = geometry_msgs::msg::TwistStamped();

                        msg.header.stamp = this->now();
                        msg.header.frame_id = "base_footprint";

                        msg.twist.linear.x = joy->axes[1] * speed;
                        msg.twist.angular.z = joy->axes[3] * speed / 3;

                        if (controlable) {
                                cmd_vel_pub->publish(msg);
                        }
                }

                rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
                rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub;

                bool controlable;
                double speed;
};


int main(int argc, char * argv[]){
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<CarController>());
        rclcpp::shutdown();
        return 0;
}
