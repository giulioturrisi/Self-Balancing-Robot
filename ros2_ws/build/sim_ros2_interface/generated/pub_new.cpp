    else if(in->topicType == "builtin_interfaces/msg/Duration")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<builtin_interfaces::msg::Duration>(in->topicName, qos);
    }
    else if(in->topicType == "builtin_interfaces/msg/Time")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<builtin_interfaces::msg::Time>(in->topicName, qos);
    }
    else if(in->topicType == "geometry_msgs/msg/Quaternion")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<geometry_msgs::msg::Quaternion>(in->topicName, qos);
    }
    else if(in->topicType == "geometry_msgs/msg/Transform")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<geometry_msgs::msg::Transform>(in->topicName, qos);
    }
    else if(in->topicType == "geometry_msgs/msg/TransformStamped")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<geometry_msgs::msg::TransformStamped>(in->topicName, qos);
    }
    else if(in->topicType == "geometry_msgs/msg/Vector3")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<geometry_msgs::msg::Vector3>(in->topicName, qos);
    }
    else if(in->topicType == "sensor_msgs/msg/Image")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<sensor_msgs::msg::Image>(in->topicName, qos);
    }
    else if(in->topicType == "sensor_msgs/msg/PointCloud2")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(in->topicName, qos);
    }
    else if(in->topicType == "sensor_msgs/msg/PointField")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<sensor_msgs::msg::PointField>(in->topicName, qos);
    }
    else if(in->topicType == "std_msgs/msg/Bool")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::Bool>(in->topicName, qos);
    }
    else if(in->topicType == "std_msgs/msg/Byte")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::Byte>(in->topicName, qos);
    }
    else if(in->topicType == "std_msgs/msg/ColorRGBA")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::ColorRGBA>(in->topicName, qos);
    }
    else if(in->topicType == "std_msgs/msg/Empty")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::Empty>(in->topicName, qos);
    }
    else if(in->topicType == "std_msgs/msg/Float32")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::Float32>(in->topicName, qos);
    }
    else if(in->topicType == "std_msgs/msg/Float64")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::Float64>(in->topicName, qos);
    }
    else if(in->topicType == "std_msgs/msg/Header")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::Header>(in->topicName, qos);
    }
    else if(in->topicType == "std_msgs/msg/Int16")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::Int16>(in->topicName, qos);
    }
    else if(in->topicType == "std_msgs/msg/Int32")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::Int32>(in->topicName, qos);
    }
    else if(in->topicType == "std_msgs/msg/Int64")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::Int64>(in->topicName, qos);
    }
    else if(in->topicType == "std_msgs/msg/Int8")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::Int8>(in->topicName, qos);
    }
    else if(in->topicType == "std_msgs/msg/String")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::String>(in->topicName, qos);
    }
    else if(in->topicType == "std_msgs/msg/UInt16")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::UInt16>(in->topicName, qos);
    }
    else if(in->topicType == "std_msgs/msg/UInt32")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::UInt32>(in->topicName, qos);
    }
    else if(in->topicType == "std_msgs/msg/UInt64")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::UInt64>(in->topicName, qos);
    }
    else if(in->topicType == "std_msgs/msg/UInt8")
    {
        rclcpp::QoS qos = 10;
        publisherProxy->publisher = node->create_publisher<std_msgs::msg::UInt8>(in->topicName, qos);
    }
