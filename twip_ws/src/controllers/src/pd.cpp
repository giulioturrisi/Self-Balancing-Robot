#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

#include "tf2/utils.h"

using std::placeholders::_1;

double state_k[6];
double velocity_wheel_left = 0;
double velocity_wheel_right = 0;

class PDController : public rclcpp::Node
{
  public:
    
    PDController()
    : Node("pd_controller")
    {
      publisher_motor_left = this->create_publisher<std_msgs::msg::Float64>("leftMotor", 10);
      publisher_motor_right = this->create_publisher<std_msgs::msg::Float64>("rightMotor", 10);

      publisher_step = this->create_publisher<std_msgs::msg::Bool>("triggerNextStep", 10);

      subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "tf", 10, std::bind(&PDController::topic_callback, this, _1));

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
    
    
    void topic_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) const
    {

      tf2::Quaternion q(
        msg->transforms[0].transform.rotation.x,
        msg->transforms[0].transform.rotation.y,
        msg->transforms[0].transform.rotation.z,
        msg->transforms[0].transform.rotation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
     
      //velocity and position com
      state_k[1] = (msg->transforms[0].transform.translation.x - state_k[0])/dt;
      state_k[0] = msg->transforms[0].transform.translation.x;

      //angular velocity and angular position com pitch
      state_k[3] = (pitch - state_k[2])/dt;  
      state_k[2] = pitch;
      //what we called theta
      RCLCPP_INFO(this->get_logger(), "theta: '%f'", pitch);
      RCLCPP_INFO(this->get_logger(), "theta_dot: '%f'", state_k[3]);
      RCLCPP_INFO(this->get_logger(), "x: '%f'", state_k[0]);
      RCLCPP_INFO(this->get_logger(), "x_dot: '%f'", state_k[1]);



      //angular velocity and angular position com yaw
      state_k[5] = (yaw - state_k[4])/dt;  
      state_k[4] = yaw;




      //control inputs message
      auto tau_left = std_msgs::msg::Float64();
      auto tau_right = std_msgs::msg::Float64();

      //gain without position com
      tau_left.data = -(10*state_k[2] + 2.5516*state_k[3] + 2*(-1 + state_k[0]) + 1*state_k[1]);
      tau_right.data = -(10*state_k[2] + 2.5516*state_k[3] + 2*(-1 + state_k[0]) + 1*state_k[1]);

    
      //publishing the control input
      publisher_motor_left->publish(tau_left);
      publisher_motor_right->publish(tau_right);


      RCLCPP_INFO(this->get_logger(), "u left '%f'", tau_left.data);
      RCLCPP_INFO(this->get_logger(), "u right '%f'", tau_right.data);


      //asking to CoppeliaSim the next step
      auto step = std_msgs::msg::Bool();
      step.data = true;
      publisher_step->publish(step);


    }

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_motor_left;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_motor_right;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_step;

    double dt = 0.01;


    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PDController>());
  rclcpp::shutdown();
  return 0;
}



