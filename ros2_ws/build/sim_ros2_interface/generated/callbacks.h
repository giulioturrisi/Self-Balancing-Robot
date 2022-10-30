#ifndef SIM_ROS2_PLUGIN__CALLBACKS__H
#define SIM_ROS2_PLUGIN__CALLBACKS__H

#include <ros_msg_builtin_io.h>
#include <sim_ros2_interface.h>

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <example_interfaces/action/fibonacci.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/byte.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

void write__builtin_interfaces__msg__Duration(const builtin_interfaces::msg::Duration& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__builtin_interfaces__msg__Duration(int stack, builtin_interfaces::msg::Duration *msg, const ROS2ReadOptions *opt = NULL);
void write__builtin_interfaces__msg__Time(const builtin_interfaces::msg::Time& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__builtin_interfaces__msg__Time(int stack, builtin_interfaces::msg::Time *msg, const ROS2ReadOptions *opt = NULL);
void write__example_interfaces__action__Fibonacci__Goal(const example_interfaces::action::Fibonacci::Goal& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__example_interfaces__action__Fibonacci__Goal(int stack, example_interfaces::action::Fibonacci::Goal *msg, const ROS2ReadOptions *opt = NULL);
void write__example_interfaces__action__Fibonacci__Feedback(const example_interfaces::action::Fibonacci::Feedback& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__example_interfaces__action__Fibonacci__Feedback(int stack, example_interfaces::action::Fibonacci::Feedback *msg, const ROS2ReadOptions *opt = NULL);
void write__example_interfaces__action__Fibonacci__Result(const example_interfaces::action::Fibonacci::Result& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__example_interfaces__action__Fibonacci__Result(int stack, example_interfaces::action::Fibonacci::Result *msg, const ROS2ReadOptions *opt = NULL);
void write__example_interfaces__srv__AddTwoInts__Request(const example_interfaces::srv::AddTwoInts::Request& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__example_interfaces__srv__AddTwoInts__Request(int stack, example_interfaces::srv::AddTwoInts::Request *msg, const ROS2ReadOptions *opt = NULL);
void write__example_interfaces__srv__AddTwoInts__Response(const example_interfaces::srv::AddTwoInts::Response& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__example_interfaces__srv__AddTwoInts__Response(int stack, example_interfaces::srv::AddTwoInts::Response *msg, const ROS2ReadOptions *opt = NULL);
void write__geometry_msgs__msg__Quaternion(const geometry_msgs::msg::Quaternion& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__geometry_msgs__msg__Quaternion(int stack, geometry_msgs::msg::Quaternion *msg, const ROS2ReadOptions *opt = NULL);
void write__geometry_msgs__msg__Transform(const geometry_msgs::msg::Transform& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__geometry_msgs__msg__Transform(int stack, geometry_msgs::msg::Transform *msg, const ROS2ReadOptions *opt = NULL);
void write__geometry_msgs__msg__TransformStamped(const geometry_msgs::msg::TransformStamped& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__geometry_msgs__msg__TransformStamped(int stack, geometry_msgs::msg::TransformStamped *msg, const ROS2ReadOptions *opt = NULL);
void write__geometry_msgs__msg__Vector3(const geometry_msgs::msg::Vector3& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__geometry_msgs__msg__Vector3(int stack, geometry_msgs::msg::Vector3 *msg, const ROS2ReadOptions *opt = NULL);
void write__sensor_msgs__msg__Image(const sensor_msgs::msg::Image& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__sensor_msgs__msg__Image(int stack, sensor_msgs::msg::Image *msg, const ROS2ReadOptions *opt = NULL);
void write__sensor_msgs__msg__PointCloud2(const sensor_msgs::msg::PointCloud2& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__sensor_msgs__msg__PointCloud2(int stack, sensor_msgs::msg::PointCloud2 *msg, const ROS2ReadOptions *opt = NULL);
void write__sensor_msgs__msg__PointField(const sensor_msgs::msg::PointField& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__sensor_msgs__msg__PointField(int stack, sensor_msgs::msg::PointField *msg, const ROS2ReadOptions *opt = NULL);
void write__std_msgs__msg__Bool(const std_msgs::msg::Bool& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__std_msgs__msg__Bool(int stack, std_msgs::msg::Bool *msg, const ROS2ReadOptions *opt = NULL);
void write__std_msgs__msg__Byte(const std_msgs::msg::Byte& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__std_msgs__msg__Byte(int stack, std_msgs::msg::Byte *msg, const ROS2ReadOptions *opt = NULL);
void write__std_msgs__msg__ColorRGBA(const std_msgs::msg::ColorRGBA& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__std_msgs__msg__ColorRGBA(int stack, std_msgs::msg::ColorRGBA *msg, const ROS2ReadOptions *opt = NULL);
void write__std_msgs__msg__Empty(const std_msgs::msg::Empty& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__std_msgs__msg__Empty(int stack, std_msgs::msg::Empty *msg, const ROS2ReadOptions *opt = NULL);
void write__std_msgs__msg__Float32(const std_msgs::msg::Float32& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__std_msgs__msg__Float32(int stack, std_msgs::msg::Float32 *msg, const ROS2ReadOptions *opt = NULL);
void write__std_msgs__msg__Float64(const std_msgs::msg::Float64& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__std_msgs__msg__Float64(int stack, std_msgs::msg::Float64 *msg, const ROS2ReadOptions *opt = NULL);
void write__std_msgs__msg__Header(const std_msgs::msg::Header& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__std_msgs__msg__Header(int stack, std_msgs::msg::Header *msg, const ROS2ReadOptions *opt = NULL);
void write__std_msgs__msg__Int16(const std_msgs::msg::Int16& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__std_msgs__msg__Int16(int stack, std_msgs::msg::Int16 *msg, const ROS2ReadOptions *opt = NULL);
void write__std_msgs__msg__Int32(const std_msgs::msg::Int32& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__std_msgs__msg__Int32(int stack, std_msgs::msg::Int32 *msg, const ROS2ReadOptions *opt = NULL);
void write__std_msgs__msg__Int64(const std_msgs::msg::Int64& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__std_msgs__msg__Int64(int stack, std_msgs::msg::Int64 *msg, const ROS2ReadOptions *opt = NULL);
void write__std_msgs__msg__Int8(const std_msgs::msg::Int8& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__std_msgs__msg__Int8(int stack, std_msgs::msg::Int8 *msg, const ROS2ReadOptions *opt = NULL);
void write__std_msgs__msg__String(const std_msgs::msg::String& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__std_msgs__msg__String(int stack, std_msgs::msg::String *msg, const ROS2ReadOptions *opt = NULL);
void write__std_msgs__msg__UInt16(const std_msgs::msg::UInt16& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__std_msgs__msg__UInt16(int stack, std_msgs::msg::UInt16 *msg, const ROS2ReadOptions *opt = NULL);
void write__std_msgs__msg__UInt32(const std_msgs::msg::UInt32& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__std_msgs__msg__UInt32(int stack, std_msgs::msg::UInt32 *msg, const ROS2ReadOptions *opt = NULL);
void write__std_msgs__msg__UInt64(const std_msgs::msg::UInt64& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__std_msgs__msg__UInt64(int stack, std_msgs::msg::UInt64 *msg, const ROS2ReadOptions *opt = NULL);
void write__std_msgs__msg__UInt8(const std_msgs::msg::UInt8& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__std_msgs__msg__UInt8(int stack, std_msgs::msg::UInt8 *msg, const ROS2ReadOptions *opt = NULL);
void write__std_srvs__srv__Empty__Request(const std_srvs::srv::Empty::Request& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__std_srvs__srv__Empty__Request(int stack, std_srvs::srv::Empty::Request *msg, const ROS2ReadOptions *opt = NULL);
void write__std_srvs__srv__Empty__Response(const std_srvs::srv::Empty::Response& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__std_srvs__srv__Empty__Response(int stack, std_srvs::srv::Empty::Response *msg, const ROS2ReadOptions *opt = NULL);
void write__std_srvs__srv__SetBool__Request(const std_srvs::srv::SetBool::Request& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__std_srvs__srv__SetBool__Request(int stack, std_srvs::srv::SetBool::Request *msg, const ROS2ReadOptions *opt = NULL);
void write__std_srvs__srv__SetBool__Response(const std_srvs::srv::SetBool::Response& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__std_srvs__srv__SetBool__Response(int stack, std_srvs::srv::SetBool::Response *msg, const ROS2ReadOptions *opt = NULL);
void write__std_srvs__srv__Trigger__Request(const std_srvs::srv::Trigger::Request& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__std_srvs__srv__Trigger__Request(int stack, std_srvs::srv::Trigger::Request *msg, const ROS2ReadOptions *opt = NULL);
void write__std_srvs__srv__Trigger__Response(const std_srvs::srv::Trigger::Response& msg, int stack, const ROS2WriteOptions *opt = NULL);
void read__std_srvs__srv__Trigger__Response(int stack, std_srvs::srv::Trigger::Response *msg, const ROS2ReadOptions *opt = NULL);
void ros_callback__builtin_interfaces__msg__Duration(const builtin_interfaces::msg::Duration::SharedPtr msg, SubscriptionProxy *proxy);
void ros_callback__builtin_interfaces__msg__Time(const builtin_interfaces::msg::Time::SharedPtr msg, SubscriptionProxy *proxy);
void ros_callback__geometry_msgs__msg__Quaternion(const geometry_msgs::msg::Quaternion::SharedPtr msg, SubscriptionProxy *proxy);
void ros_callback__geometry_msgs__msg__Transform(const geometry_msgs::msg::Transform::SharedPtr msg, SubscriptionProxy *proxy);
void ros_callback__geometry_msgs__msg__TransformStamped(const geometry_msgs::msg::TransformStamped::SharedPtr msg, SubscriptionProxy *proxy);
void ros_callback__geometry_msgs__msg__Vector3(const geometry_msgs::msg::Vector3::SharedPtr msg, SubscriptionProxy *proxy);
void ros_callback__sensor_msgs__msg__Image(const sensor_msgs::msg::Image::SharedPtr msg, SubscriptionProxy *proxy);
void ros_callback__sensor_msgs__msg__PointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr msg, SubscriptionProxy *proxy);
void ros_callback__sensor_msgs__msg__PointField(const sensor_msgs::msg::PointField::SharedPtr msg, SubscriptionProxy *proxy);
void ros_callback__std_msgs__msg__Bool(const std_msgs::msg::Bool::SharedPtr msg, SubscriptionProxy *proxy);
void ros_callback__std_msgs__msg__Byte(const std_msgs::msg::Byte::SharedPtr msg, SubscriptionProxy *proxy);
void ros_callback__std_msgs__msg__ColorRGBA(const std_msgs::msg::ColorRGBA::SharedPtr msg, SubscriptionProxy *proxy);
void ros_callback__std_msgs__msg__Empty(const std_msgs::msg::Empty::SharedPtr msg, SubscriptionProxy *proxy);
void ros_callback__std_msgs__msg__Float32(const std_msgs::msg::Float32::SharedPtr msg, SubscriptionProxy *proxy);
void ros_callback__std_msgs__msg__Float64(const std_msgs::msg::Float64::SharedPtr msg, SubscriptionProxy *proxy);
void ros_callback__std_msgs__msg__Header(const std_msgs::msg::Header::SharedPtr msg, SubscriptionProxy *proxy);
void ros_callback__std_msgs__msg__Int16(const std_msgs::msg::Int16::SharedPtr msg, SubscriptionProxy *proxy);
void ros_callback__std_msgs__msg__Int32(const std_msgs::msg::Int32::SharedPtr msg, SubscriptionProxy *proxy);
void ros_callback__std_msgs__msg__Int64(const std_msgs::msg::Int64::SharedPtr msg, SubscriptionProxy *proxy);
void ros_callback__std_msgs__msg__Int8(const std_msgs::msg::Int8::SharedPtr msg, SubscriptionProxy *proxy);
void ros_callback__std_msgs__msg__String(const std_msgs::msg::String::SharedPtr msg, SubscriptionProxy *proxy);
void ros_callback__std_msgs__msg__UInt16(const std_msgs::msg::UInt16::SharedPtr msg, SubscriptionProxy *proxy);
void ros_callback__std_msgs__msg__UInt32(const std_msgs::msg::UInt32::SharedPtr msg, SubscriptionProxy *proxy);
void ros_callback__std_msgs__msg__UInt64(const std_msgs::msg::UInt64::SharedPtr msg, SubscriptionProxy *proxy);
void ros_callback__std_msgs__msg__UInt8(const std_msgs::msg::UInt8::SharedPtr msg, SubscriptionProxy *proxy);
bool ros_srv_callback__example_interfaces__srv__AddTwoInts(const std::shared_ptr<rmw_request_id_t> request_header, const example_interfaces::srv::AddTwoInts::Request::SharedPtr req, example_interfaces::srv::AddTwoInts::Response::SharedPtr res, ServiceProxy *proxy);
bool ros_srv_callback__std_srvs__srv__Empty(const std::shared_ptr<rmw_request_id_t> request_header, const std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res, ServiceProxy *proxy);
bool ros_srv_callback__std_srvs__srv__SetBool(const std::shared_ptr<rmw_request_id_t> request_header, const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res, ServiceProxy *proxy);
bool ros_srv_callback__std_srvs__srv__Trigger(const std::shared_ptr<rmw_request_id_t> request_header, const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res, ServiceProxy *proxy);
void ros_action_callback__example_interfaces__action__Fibonacci__Feedback(int scriptID, const char *callback, const rclcpp_action::GoalUUID &goal_id, const example_interfaces::action::Fibonacci::Feedback *feedback, ActionClientProxy *proxy);
void ros_action_callback__example_interfaces__action__Fibonacci__Result(int scriptID, const char *callback, const rclcpp_action::GoalUUID &goal_id, int action_result_code, const example_interfaces::action::Fibonacci::Result::SharedPtr result, ActionClientProxy *proxy);
rclcpp_action::GoalResponse ros_action_callback__handle_goal__example_interfaces__action__Fibonacci__Goal(int scriptID, const char *callback, const rclcpp_action::GoalUUID &goal_id, const example_interfaces::action::Fibonacci::Goal *goal, ActionServerProxy *proxy);
rclcpp_action::CancelResponse ros_action_callback__handle_cancel__example_interfaces__action__Fibonacci__Goal(int scriptID, const char *callback, const rclcpp_action::GoalUUID &goal_id, const example_interfaces::action::Fibonacci::Goal *goal, ActionServerProxy *proxy);
void ros_action_callback__handle_accepted__example_interfaces__action__Fibonacci__Goal(int scriptID, const char *callback, const rclcpp_action::GoalUUID &goal_id, const example_interfaces::action::Fibonacci::Goal *goal, ActionServerProxy *proxy);

#endif // SIM_ROS2_PLUGIN__CALLBACKS__H
