#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.h"

           
//#include "geometry_msgs/msg/TrasformStamped.hpp"

#include "tf2/utils.h"

using std::placeholders::_1;


class ComplementaryFilter : public rclcpp::Node
{
  public:
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_tf;



    ComplementaryFilter()
    : Node("complementary_filter")
    {

      subscription_imu = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 1, std::bind(&ComplementaryFilter::estimator_callback, this, _1));

      publisher_tf = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", 1);

      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);



    }

  private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    
    void estimator_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
    {


        //USELESS BUT ONE DAY KALMAN


        double lin_acc_x = 0, lin_acc_y = 0, lin_acc_z = 0;
        double ang_acc_x = 0, ang_acc_y = 0, ang_acc_z = 0;

        double orient_x = 0, orient_y = 0, orient_z = 0;

        lin_acc_x = msg->linear_acceleration.x;
        lin_acc_y = msg->linear_acceleration.y;
        lin_acc_z = msg->linear_acceleration.z;

        ang_acc_x = msg->angular_velocity.x;
        ang_acc_y = msg->angular_velocity.y;
        ang_acc_z = msg->angular_velocity.z;

        orient_x = msg->orientation.x;
        orient_y = msg->orientation.y;
        orient_z = msg->orientation.z;
        //static tf2_ros::TransformBroadcaster br;
        geometry_msgs::msg::TransformStamped transformStamped;

        auto tf = tf2_msgs::msg::TFMessage();

        //RCLCPP_INFO(this->get_logger(), "orient_x: '%f'", orient_x);

        //tf.transforms[0].transform.rotation.x = orient_x;

        //RCLCPP_INFO(this->get_logger(), "orient_x2: '%f'", orient_x);
        
        transformStamped.transform.rotation.x = orient_x;
        transformStamped.transform.rotation.y = orient_y;
        transformStamped.transform.rotation.z = orient_z;


        tf_broadcaster_->sendTransform(transformStamped);
        //publisher_tf->publish(tf);
   
    }



    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ComplementaryFilter>());
  rclcpp::shutdown();
  return 0;
}



