#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "std_msgs/msg/float64.hpp"

#include "tf2/utils.h"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      publisher_motor_left = this->create_publisher<std_msgs::msg::Float64>("leftMotor", 10);
      publisher_motor_right = this->create_publisher<std_msgs::msg::Float64>("rightMotor", 10);

      subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "tf", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));


    }

  private:
    void topic_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) const
    {
      //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      RCLCPP_INFO(this->get_logger(), "translation: '%f'", msg->transforms[0].transform.translation.x);
      RCLCPP_INFO(this->get_logger(), "rotation: '%f'", msg->transforms[0].transform.rotation.x);

      tf2::Quaternion q(
        msg->transforms[0].transform.rotation.x,
        msg->transforms[0].transform.rotation.y,
        msg->transforms[0].transform.rotation.z,
        msg->transforms[0].transform.rotation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      RCLCPP_INFO(this->get_logger(), "roll: '%f'", roll);
      //what we called theta
      RCLCPP_INFO(this->get_logger(), "pitch: '%f'", pitch);
      RCLCPP_INFO(this->get_logger(), "yaw: '%f'", yaw);

      float pitch_dot = (pitch - last_pitch)/0.10;  


      auto tau_left = std_msgs::msg::Float64();
      auto tau_right = std_msgs::msg::Float64();

      //k_lqr
      //0.4572    0.5516    3.1212    4.5492
      //0.4572    0.5516   -3.1212   -4.5492
      tau_left.data = -(0.4572*pitch + 0.5516*pitch_dot + 0 + 0);
      tau_right.data = -(0.4572*pitch + 0.5516*pitch_dot + 0 + 0);

      publisher_motor_left->publish(tau_left);
      publisher_motor_right->publish(tau_right);



    }
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_motor_left;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_motor_right;
    
    float last_pitch = 0;
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}