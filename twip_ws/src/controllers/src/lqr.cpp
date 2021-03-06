#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "tf2/utils.h"

using std::placeholders::_1;

double state_k[6];
double velocity_wheel_left = 0;
double velocity_wheel_right = 0;
double go_straight = 0;
double turn = 0;
double state_d_pos = 0;

class LqrController : public rclcpp::Node
{
  public:
    
    LqrController()
    : Node("lqr_controller")
    {
      publisher_motor_left = this->create_publisher<std_msgs::msg::Float64>("leftMotor", 10);
      publisher_motor_right = this->create_publisher<std_msgs::msg::Float64>("rightMotor", 10);

      publisher_step = this->create_publisher<std_msgs::msg::Bool>("triggerNextStep", 10);

      subscription_controller = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "tf", 10, std::bind(&LqrController::controller_callback, this, _1));

      subscription_direction = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1, std::bind(&LqrController::direction_callback, this, _1));

      //initial state
      state_k[0] = 0.0; //x
      state_k[1] = 0.0; //x_dot
      state_k[2] = 0.17; //theta
      state_k[3] = 0.0; //theta_dot
      state_k[4] = 0; //phi
      state_k[5] = 0.0; //phi_dot



    }

  private:

    void forward_dynamic_fun(double u_l, double u_r, const double state[6], double
      *theta_ddot, double *phi_ddot, double *x_ddot) const
    {
      double b_theta_ddot_tmp;
      double theta_ddot_tmp;
      double theta_ddot_tmp_tmp;

      theta_ddot_tmp_tmp = std::cos(state[2]);
      theta_ddot_tmp = u_l + u_r;
      b_theta_ddot_tmp = 11.187452640000002 * std::sin(state[2]);
      *theta_ddot = (1.826971 * b_theta_ddot_tmp - 1.1415768 * theta_ddot_tmp_tmp *
                    theta_ddot_tmp / 0.1) / (0.006107564053 - 1.30319759029824 *
        theta_ddot_tmp_tmp * theta_ddot_tmp_tmp);
      *phi_ddot = 0.35 * ((u_l - u_r) / 0.1) / 1.228343;
      *x_ddot = (theta_ddot_tmp / 0.1 - 1.1415768 * std::cos(state[2]) *
                b_theta_ddot_tmp / 0.005928) / (1.826971 - 1.30319759029824 *
        (theta_ddot_tmp_tmp * theta_ddot_tmp_tmp) / 0.005928);
    }
    
    
    void controller_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) const
    {
      //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      //RCLCPP_INFO(this->get_logger(), "translation: '%f'", msg->transforms[0].transform.translation.x);
      //RCLCPP_INFO(this->get_logger(), "rotation: '%f'", msg->transforms[0].transform.rotation.x);

      tf2::Quaternion q(
        msg->transforms[0].transform.rotation.x,
        msg->transforms[0].transform.rotation.y,
        msg->transforms[0].transform.rotation.z,
        msg->transforms[0].transform.rotation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
     
      state_k[1] = (msg->transforms[0].transform.translation.x - state_k[0])/dt;
      state_k[0] = msg->transforms[0].transform.translation.x;

      state_k[3] = (pitch - state_k[2])/dt;  
      state_k[2] = pitch;
      

      state_k[5] = (yaw - state_k[4])/dt;  
      state_k[4] = yaw;
      //what we called theta
      RCLCPP_INFO(this->get_logger(), "theta: '%f'", pitch);
      //RCLCPP_INFO(this->get_logger(), "theta_dot: '%f'", state_k[3]);

      //RCLCPP_INFO(this->get_logger(), "yaw: '%f'", yaw);
      //RCLCPP_INFO(this->get_logger(), "yaw_dot: '%f'", state_k[5]);

      //RCLCPP_INFO(this->get_logger(), "x: '%f'", state_k[0]);
      //RCLCPP_INFO(this->get_logger(), "x_dot: '%f'", state_k[1]);


      auto tau_left = std_msgs::msg::Float64();
      auto tau_right = std_msgs::msg::Float64();

      if(go_straight != 0) {
        double sliding = 50*(state_k[2] - go_straight/10) + state_k[3]; 
        RCLCPP_INFO(this->get_logger(), "SLIDING: '%f'", sliding);
        RCLCPP_INFO(this->get_logger(), "theta_desired: '%f'", go_straight/10);
        tau_left.data = 0.3025*go_straight*2 -(2.2853*(state_k[2] - go_straight/10) + 0.7676*state_k[3] + 0.9576*(state_k[4] - turn) + 0.8015*state_k[5]);
        tau_right.data = 0.3025*go_straight*2 -(2.2853*(state_k[2] - go_straight/10) + 0.7676*state_k[3] - 0.9576*(state_k[4] - turn) - 0.8015*state_k[5]);

        tau_left.data = tau_left.data - 5*tanh(sliding);
        tau_right.data = tau_right.data - 5*tanh(sliding);
      }
      else {
        double sliding = 20*(state_k[0] - state_d_pos) + (state_k[1]) + 20*(state_k[2]) + state_k[3]; 
        tau_left.data = -(2.2361*(state_k[0] - state_d_pos) +  3.7439*state_k[1] + 11.5324*state_k[2] + 4.3374*state_k[3] + 0.7071*(state_k[4] - turn) + 1.3630*state_k[5]);
        tau_right.data = -(2.2361*(state_k[0] - state_d_pos) +  3.7439*state_k[1] + 11.5324*state_k[2] + 4.3374*state_k[3] - 0.7071*(state_k[4] - turn) - 1.3630*state_k[5]);
      }

      //k_lqr without position
      //0.4572    0.5516    3.1212    4.5492
      //0.4572    0.5516   -3.1212   -4.5492
      //tau_left.data = -(0.4572*state_k[2] + 0.5516*state_k[3] + 0 + 0);
      //tau_right.data = -(0.4572*state_k[2] + 0.5516*state_k[3] + 0 + 0);

      //k_lqr with position
      //0.7071    1.5194    6.4885    3.8998    3.1623    5.3976
      //0.7071    1.4116    8.6997    3.4671   -3.1623   -9.9345
      //tau_left.data = -(0.7071*(state_k[0] - 3) +  1.5194*state_k[1] + 6.4885*state_k[2] + 3.8998*state_k[3] + 0.3162*(state_k[4] - 0) + 0.1397*state_k[5]);
      //tau_right.data = -(0.7071*(state_k[0] - 3) +  1.5194*state_k[1] + 6.4885*state_k[2] + 3.8998*state_k[3] - 0.3162*(state_k[4] - 0) - 0.1397*state_k[5]);


      publisher_motor_left->publish(tau_left);
      publisher_motor_right->publish(tau_right);

      //try to calculate velocity from dynamic model and pass to coppeliasim
      //double phi_ddot, theta_ddot, x_ddot;
      //forward_dynamic_fun(tau_right.data, tau_left.data, state_k, &theta_ddot, &phi_ddot, &x_ddot);
      //RCLCPP_INFO(this->get_logger(), "theta_ddot: '%f'", theta_ddot);



      auto step = std_msgs::msg::Bool();
      step.data = true;
      publisher_step->publish(step);



    }

void direction_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "straight: '%f'", msg->linear.x);
      RCLCPP_INFO(this->get_logger(), "turn: '%f'", msg->angular.z);
      if(go_straight != msg->linear.x) {
        state_d_pos = state_k[0];
      }
      go_straight = msg->linear.x;
      turn = msg->angular.z;

    }

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_controller;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_direction;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_motor_left;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_motor_right;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_step;

    double dt = 0.01;


    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LqrController>());
  rclcpp::shutdown();
  return 0;
}



