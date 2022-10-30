    else if(in->topicType == "builtin_interfaces/msg/Duration")
    {
        auto cb = [=](const builtin_interfaces::msg::Duration::SharedPtr msg) { ros_callback__builtin_interfaces__msg__Duration(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<builtin_interfaces::msg::Duration>(in->topicName, qos, cb);
    }
    else if(in->topicType == "builtin_interfaces/msg/Time")
    {
        auto cb = [=](const builtin_interfaces::msg::Time::SharedPtr msg) { ros_callback__builtin_interfaces__msg__Time(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<builtin_interfaces::msg::Time>(in->topicName, qos, cb);
    }
    else if(in->topicType == "geometry_msgs/msg/Quaternion")
    {
        auto cb = [=](const geometry_msgs::msg::Quaternion::SharedPtr msg) { ros_callback__geometry_msgs__msg__Quaternion(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<geometry_msgs::msg::Quaternion>(in->topicName, qos, cb);
    }
    else if(in->topicType == "geometry_msgs/msg/Transform")
    {
        auto cb = [=](const geometry_msgs::msg::Transform::SharedPtr msg) { ros_callback__geometry_msgs__msg__Transform(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<geometry_msgs::msg::Transform>(in->topicName, qos, cb);
    }
    else if(in->topicType == "geometry_msgs/msg/TransformStamped")
    {
        auto cb = [=](const geometry_msgs::msg::TransformStamped::SharedPtr msg) { ros_callback__geometry_msgs__msg__TransformStamped(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<geometry_msgs::msg::TransformStamped>(in->topicName, qos, cb);
    }
    else if(in->topicType == "geometry_msgs/msg/Vector3")
    {
        auto cb = [=](const geometry_msgs::msg::Vector3::SharedPtr msg) { ros_callback__geometry_msgs__msg__Vector3(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<geometry_msgs::msg::Vector3>(in->topicName, qos, cb);
    }
    else if(in->topicType == "sensor_msgs/msg/Image")
    {
        auto cb = [=](const sensor_msgs::msg::Image::SharedPtr msg) { ros_callback__sensor_msgs__msg__Image(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<sensor_msgs::msg::Image>(in->topicName, qos, cb);
    }
    else if(in->topicType == "sensor_msgs/msg/PointCloud2")
    {
        auto cb = [=](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { ros_callback__sensor_msgs__msg__PointCloud2(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(in->topicName, qos, cb);
    }
    else if(in->topicType == "sensor_msgs/msg/PointField")
    {
        auto cb = [=](const sensor_msgs::msg::PointField::SharedPtr msg) { ros_callback__sensor_msgs__msg__PointField(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<sensor_msgs::msg::PointField>(in->topicName, qos, cb);
    }
    else if(in->topicType == "std_msgs/msg/Bool")
    {
        auto cb = [=](const std_msgs::msg::Bool::SharedPtr msg) { ros_callback__std_msgs__msg__Bool(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::Bool>(in->topicName, qos, cb);
    }
    else if(in->topicType == "std_msgs/msg/Byte")
    {
        auto cb = [=](const std_msgs::msg::Byte::SharedPtr msg) { ros_callback__std_msgs__msg__Byte(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::Byte>(in->topicName, qos, cb);
    }
    else if(in->topicType == "std_msgs/msg/ColorRGBA")
    {
        auto cb = [=](const std_msgs::msg::ColorRGBA::SharedPtr msg) { ros_callback__std_msgs__msg__ColorRGBA(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::ColorRGBA>(in->topicName, qos, cb);
    }
    else if(in->topicType == "std_msgs/msg/Empty")
    {
        auto cb = [=](const std_msgs::msg::Empty::SharedPtr msg) { ros_callback__std_msgs__msg__Empty(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::Empty>(in->topicName, qos, cb);
    }
    else if(in->topicType == "std_msgs/msg/Float32")
    {
        auto cb = [=](const std_msgs::msg::Float32::SharedPtr msg) { ros_callback__std_msgs__msg__Float32(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::Float32>(in->topicName, qos, cb);
    }
    else if(in->topicType == "std_msgs/msg/Float64")
    {
        auto cb = [=](const std_msgs::msg::Float64::SharedPtr msg) { ros_callback__std_msgs__msg__Float64(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::Float64>(in->topicName, qos, cb);
    }
    else if(in->topicType == "std_msgs/msg/Header")
    {
        auto cb = [=](const std_msgs::msg::Header::SharedPtr msg) { ros_callback__std_msgs__msg__Header(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::Header>(in->topicName, qos, cb);
    }
    else if(in->topicType == "std_msgs/msg/Int16")
    {
        auto cb = [=](const std_msgs::msg::Int16::SharedPtr msg) { ros_callback__std_msgs__msg__Int16(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::Int16>(in->topicName, qos, cb);
    }
    else if(in->topicType == "std_msgs/msg/Int32")
    {
        auto cb = [=](const std_msgs::msg::Int32::SharedPtr msg) { ros_callback__std_msgs__msg__Int32(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::Int32>(in->topicName, qos, cb);
    }
    else if(in->topicType == "std_msgs/msg/Int64")
    {
        auto cb = [=](const std_msgs::msg::Int64::SharedPtr msg) { ros_callback__std_msgs__msg__Int64(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::Int64>(in->topicName, qos, cb);
    }
    else if(in->topicType == "std_msgs/msg/Int8")
    {
        auto cb = [=](const std_msgs::msg::Int8::SharedPtr msg) { ros_callback__std_msgs__msg__Int8(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::Int8>(in->topicName, qos, cb);
    }
    else if(in->topicType == "std_msgs/msg/String")
    {
        auto cb = [=](const std_msgs::msg::String::SharedPtr msg) { ros_callback__std_msgs__msg__String(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::String>(in->topicName, qos, cb);
    }
    else if(in->topicType == "std_msgs/msg/UInt16")
    {
        auto cb = [=](const std_msgs::msg::UInt16::SharedPtr msg) { ros_callback__std_msgs__msg__UInt16(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::UInt16>(in->topicName, qos, cb);
    }
    else if(in->topicType == "std_msgs/msg/UInt32")
    {
        auto cb = [=](const std_msgs::msg::UInt32::SharedPtr msg) { ros_callback__std_msgs__msg__UInt32(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::UInt32>(in->topicName, qos, cb);
    }
    else if(in->topicType == "std_msgs/msg/UInt64")
    {
        auto cb = [=](const std_msgs::msg::UInt64::SharedPtr msg) { ros_callback__std_msgs__msg__UInt64(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::UInt64>(in->topicName, qos, cb);
    }
    else if(in->topicType == "std_msgs/msg/UInt8")
    {
        auto cb = [=](const std_msgs::msg::UInt8::SharedPtr msg) { ros_callback__std_msgs__msg__UInt8(msg, subscriptionProxy); };
        rclcpp::QoS qos = 10;
        subscriptionProxy->subscription = node->create_subscription<std_msgs::msg::UInt8>(in->topicName, qos, cb);
    }
