#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"


#include "geometry_msgs/msg/twist.hpp"

#include "tf2/utils.h"

using std::placeholders::_1;

double state_k[6];
double velocity_wheel_left = 0;
double velocity_wheel_right = 0;
double go_straight = 0;
double turn = 0;
double state_d_pos = 0;
double velocity_last = 0, velocity_last_last = 0 , position_last = 0, position_last_last = 0;
double error_integral = 0;
class LqrController : public rclcpp::Node
{
  public:
    
    LqrController()
    : Node("lqr_controller")
    {
      publisher_motor_left = this->create_publisher<std_msgs::msg::Float64>("leftMotor", 1);
      publisher_motor_right = this->create_publisher<std_msgs::msg::Float64>("rightMotor", 1);

      publisher_motors = this->create_publisher<std_msgs::msg::Float64MultiArray>("motors", 1);

      publisher_step = this->create_publisher<std_msgs::msg::Bool>("triggerNextStep", 1);

      subscription_controller = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "tf", 1, std::bind(&LqrController::controller_callback, this, _1));
      subscription_direction = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1, std::bind(&LqrController::direction_callback, this, _1));

      //initial state
      state_k[0] = 0.0; //x
      state_k[1] = 0.0; //x_dot
      state_k[2] = 0.0; //theta
      state_k[3] = 0.0; //theta_dot
      state_k[4] = 0; //phi
      state_k[5] = 0.0; //phi_dot



    }

  private:

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_controller;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_direction;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_motor_left;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_motor_right;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_motors;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_step;

    double dt = 0.001;
    
    void controller_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) const
    {

      tf2::Quaternion q(
        msg->transforms[0].transform.rotation.x,
        msg->transforms[0].transform.rotation.y,
        msg->transforms[0].transform.rotation.z,
        msg->transforms[0].transform.rotation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      int pitch_rounded = 0; 
      
      //m.getRPY(roll, pitch, yaw);
      pitch = msg->transforms[0].transform.rotation.x;
      yaw = msg->transforms[0].transform.rotation.z;
 
      state_k[1] = (msg->transforms[0].transform.translation.x - state_k[0])/dt;
      state_k[0] = msg->transforms[0].transform.translation.x;
      

      //filter position
      pitch_rounded = pitch*100;
      RCLCPP_INFO(this->get_logger(), "pitch_rounded: '%i'", pitch_rounded);
      pitch = pitch_rounded/100.;
      RCLCPP_INFO(this->get_logger(), "pitch: '%f'", pitch);
      position_last_last = position_last;
      position_last = state_k[2];
      state_k[2] = (pitch + state_k[2] + position_last_last*0)/2.;


      velocity_last_last = velocity_last;
      velocity_last = state_k[3];
      state_k[3] = (pitch - state_k[2])/dt;  
      //filter velocity
      state_k[3] = (state_k[3] + velocity_last + velocity_last_last*0)/2.;
      state_k[3] = state_k[3];
      

      

      state_k[5] = (yaw - state_k[4])/dt;  
      state_k[4] = yaw;

      RCLCPP_INFO(this->get_logger(), "theta: '%f'", state_k[2]);
      RCLCPP_INFO(this->get_logger(), "theta dot:  '%f'", state_k[3]);
      //RCLCPP_INFO(this->get_logger(), "#######  '%f'", 0);


      auto tau_left = std_msgs::msg::Float64();
      auto tau_right = std_msgs::msg::Float64();

      if(go_straight != 0) {
        //double sliding = 50*(state_k[2] - go_straight/10) + state_k[3]; 
        //tau_left.data = 0.3025*go_straight*2 -(2.2853*(state_k[2] - go_straight/10) + 0.7676*state_k[3] + 0.9576*(state_k[4] - turn) + 0.8015*state_k[5]);
        //tau_right.data = 0.3025*go_straight*2 -(2.2853*(state_k[2] - go_straight/10) + 0.7676*state_k[3] - 0.9576*(state_k[4] - turn) - 0.8015*state_k[5]);

        //tau_left.data = tau_left.data - 5*tanh(sliding);
        //tau_right.data = tau_right.data - 5*tanh(sliding);
      }
      else {
        //double sliding = 20*(state_k[0] - state_d_pos) + (state_k[1]) + 20*(state_k[2]) + state_k[3];
        //double sliding =  50*(state_k[2]) + state_k[3]; 
        //tau_left.data = -(2.2361*(state_k[0] - state_d_pos)*0 +  3.7439*state_k[1]*0 + 11.5324*state_k[2] + 4.3374*state_k[3] + 0.7071*(state_k[4] - turn)*0 + 1.3630*state_k[5]*0);
        //tau_left.data = 11.5324*state_k[2];
        //tau_right.data = 11.5324*state_k[2];
        //tau_right.data = -(2.2361*(state_k[0] - state_d_pos)*0 +  3.7439*state_k[1]*0 + 11.5324*state_k[2] + 4.3374*state_k[3] - 0.7071*(state_k[4] - turn)*0 - 1.3630*state_k[5]*0);
        //tau_left.data = tau_left.data - 200*tanh(sliding);
        error_integral += state_k[2];
        tau_left.data = -(6000*state_k[2] + 1*state_k[3]) - 0.*error_integral;
        tau_right.data = -(6000*state_k[2] + 1*state_k[3]) - 0.*error_integral;
        //tau_right.data = tau_right.data - 200*tanh(sliding);
      }

      RCLCPP_INFO(this->get_logger(), "tau:  '%f'", tau_right.data);


      publisher_motor_left->publish(tau_left);
      publisher_motor_right->publish(tau_right);


      auto tau = std_msgs::msg::Float64MultiArray();

      tau.data.push_back(tau_left.data);
      tau.data.push_back(tau_right.data);
      publisher_motors->publish(tau);



      auto step = std_msgs::msg::Bool();
      step.data = true;
      publisher_step->publish(step);

    }

void direction_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
      if(go_straight != msg->linear.x) {
        state_d_pos = state_k[0];
      }
      go_straight = msg->linear.x;
      turn = msg->angular.z;
    }

    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LqrController>());
  rclcpp::shutdown();
  return 0;
}



