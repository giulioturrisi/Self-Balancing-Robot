#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>

#include <pigpio.h>

#include "rotary_encoder.hpp"
#include <chrono>
#include <unistd.h>

#include <cmath>

#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "tf2/utils.h"




using std::placeholders::_1;

int tickLeft = 0;
int tickRight = 0;

class MotorController : public rclcpp::Node{
  public:
    

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_motors_commands;


    MotorController()
    : Node("lqr_controller")
    {
        publisher_motors_commands = this->create_publisher<std_msgs::msg::Float64MultiArray>("motors", 1);


        //if (gpioInitialise() < 0) return 1;
        gpioSetMode(20, PI_OUTPUT);
        gpioWrite(20,0);
        gpioSetMode(26, PI_OUTPUT);
        gpioWrite(26,0);


        gpioSetMode(13, PI_OUTPUT);
        gpioSetPWMfrequency(13,1000);
        gpioSetPWMrange(13,255);
        
        

        re_decoder dec(27, 17, update_leftEncoder);

    }

  private:

    static void update_leftEncoder(const int way){
        tickLeft += way;
    }
    static void update_rightEncoder(const int way){
        tickRight += way;
    }


    void forward(bool isLeft,double duty_cycle_forw) const{
        if(isLeft == 1) {
            //left
            gpioPWM(13,duty_cycle_forw);
            gpioWrite(26,1);
            gpioWrite(20,0);
        }
        else{

        }
    }


    void backward(bool isLeft,double duty_cycle_back) const{
        if(isLeft == 1) {
            //left
            gpioPWM(13,duty_cycle_back);
            gpioWrite(26,0);
            gpioWrite(20,1);
        }
        else{

        }
    }

    
    void controller_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) const{
        
        int W = 10;
        
        double duty_cycle_Left = 0;
        double vel_d_Left = msg->data[0];

        

        double speed_Left = 0;
        double speed_last_Left = 0;
        double speed_filtered_Left = 0;

        double error_vel_Left = 0;
        double error_acc_Left = 0;
        double error_vel_integral_Left = 0;

        double angle_rad_Left = 0;
        double old_angle_rad_Left = 0;


        double duty_cycle_Right = 0;
        double vel_d_Right = msg->data[1];

        double speed_Right = 0;
        double speed_last_Right = 0;
        double speed_filtered_Right = 0;

        double error_vel_Right = 0;
        double error_acc_Right = 0;
        double error_vel_integral_Right = 0;

        double angle_rad_Right = 0;
        double old_angle_rad_Right = 0;
        


        for(int i = 0; i < 10; i++) {

            auto start = std::chrono::high_resolution_clock::now();

            //LEFT
            old_angle_rad_Left = angle_rad_Left;
            angle_rad_Left = (tickLeft*0.41)*3.14159/180;

            speed_Left = (angle_rad_Left - old_angle_rad_Left)/0.001;
            

            speed_filtered_Left = (speed_last_Left + speed_Left)/2;
            speed_last_Left = speed_Left;
            
            error_vel_Left = vel_d_Left - speed_filtered_Left;
            error_acc_Left = (speed_last_Left + speed_filtered_Left)/0.001;
            error_vel_integral_Left += error_vel_Left;

            duty_cycle_Left = W*(10*error_vel_Left + 0.1*error_vel_integral_Left - 0*error_acc_Left);


            if(duty_cycle_Left > 255) duty_cycle_Left = 255;
            if(duty_cycle_Left < -255) duty_cycle_Left = -255;

        
            if(duty_cycle_Left < 0) {
                backward(1,-duty_cycle_Left);
            }
            else {
                forward(1,duty_cycle_Left);
            }

            ////////////////////////////////////////


            //RIGHT

            old_angle_rad_Right = angle_rad_Right;
            angle_rad_Right = (tickRight*0.41)*3.14159/180;

            speed_Right = (angle_rad_Right - old_angle_rad_Right)/0.001;
            

            speed_filtered_Right = (speed_last_Right + speed_Right)/2;
            speed_last_Right = speed_Right;
            
            error_vel_Right = vel_d_Right - speed_filtered_Right;
            error_acc_Right = (speed_last_Right + speed_filtered_Right)/0.001;
            error_vel_integral_Right += error_vel_Right;

            duty_cycle_Right = W*(10*error_vel_Right + 0.1*error_vel_integral_Right - 0*error_acc_Right);


            if(duty_cycle_Right > 255) duty_cycle_Right = 255;
            if(duty_cycle_Right < -255) duty_cycle_Right = -255;

        
            if(duty_cycle_Right < 0) {
                backward(0,-duty_cycle_Right);
            }
            else {
                forward(0,duty_cycle_Right);
            }

            double Ts_fake = 0;
            while (true){
                auto finish = std::chrono::high_resolution_clock::now();
                auto time = std::chrono::duration_cast<std::chrono::microseconds>(finish - start).count();
                Ts_fake = (double)time / 1e3;
                if(Ts_fake > 0.999)
                    break;
            }
        }

        tickRight = 0;
        tickLeft = 0;
    }    

    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorController>());
  rclcpp::shutdown();
  return 0;
}

