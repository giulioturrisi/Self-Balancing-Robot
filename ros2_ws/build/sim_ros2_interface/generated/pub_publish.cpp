    else if(publisherProxy->topicType == "builtin_interfaces/msg/Duration")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<builtin_interfaces::msg::Duration> > >(publisherProxy->publisher);
        builtin_interfaces::msg::Duration msg;
        read__builtin_interfaces__msg__Duration(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
    else if(publisherProxy->topicType == "builtin_interfaces/msg/Time")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<builtin_interfaces::msg::Time> > >(publisherProxy->publisher);
        builtin_interfaces::msg::Time msg;
        read__builtin_interfaces__msg__Time(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
    else if(publisherProxy->topicType == "geometry_msgs/msg/Quaternion")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<geometry_msgs::msg::Quaternion> > >(publisherProxy->publisher);
        geometry_msgs::msg::Quaternion msg;
        read__geometry_msgs__msg__Quaternion(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
    else if(publisherProxy->topicType == "geometry_msgs/msg/Transform")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<geometry_msgs::msg::Transform> > >(publisherProxy->publisher);
        geometry_msgs::msg::Transform msg;
        read__geometry_msgs__msg__Transform(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
    else if(publisherProxy->topicType == "geometry_msgs/msg/TransformStamped")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<geometry_msgs::msg::TransformStamped> > >(publisherProxy->publisher);
        geometry_msgs::msg::TransformStamped msg;
        read__geometry_msgs__msg__TransformStamped(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
    else if(publisherProxy->topicType == "geometry_msgs/msg/Vector3")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<geometry_msgs::msg::Vector3> > >(publisherProxy->publisher);
        geometry_msgs::msg::Vector3 msg;
        read__geometry_msgs__msg__Vector3(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
    else if(publisherProxy->topicType == "sensor_msgs/msg/Image")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<sensor_msgs::msg::Image> > >(publisherProxy->publisher);
        sensor_msgs::msg::Image msg;
        read__sensor_msgs__msg__Image(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
    else if(publisherProxy->topicType == "sensor_msgs/msg/PointCloud2")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<sensor_msgs::msg::PointCloud2> > >(publisherProxy->publisher);
        sensor_msgs::msg::PointCloud2 msg;
        read__sensor_msgs__msg__PointCloud2(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
    else if(publisherProxy->topicType == "sensor_msgs/msg/PointField")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<sensor_msgs::msg::PointField> > >(publisherProxy->publisher);
        sensor_msgs::msg::PointField msg;
        read__sensor_msgs__msg__PointField(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
    else if(publisherProxy->topicType == "std_msgs/msg/Bool")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<std_msgs::msg::Bool> > >(publisherProxy->publisher);
        std_msgs::msg::Bool msg;
        read__std_msgs__msg__Bool(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
    else if(publisherProxy->topicType == "std_msgs/msg/Byte")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<std_msgs::msg::Byte> > >(publisherProxy->publisher);
        std_msgs::msg::Byte msg;
        read__std_msgs__msg__Byte(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
    else if(publisherProxy->topicType == "std_msgs/msg/ColorRGBA")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<std_msgs::msg::ColorRGBA> > >(publisherProxy->publisher);
        std_msgs::msg::ColorRGBA msg;
        read__std_msgs__msg__ColorRGBA(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
    else if(publisherProxy->topicType == "std_msgs/msg/Empty")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<std_msgs::msg::Empty> > >(publisherProxy->publisher);
        std_msgs::msg::Empty msg;
        read__std_msgs__msg__Empty(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
    else if(publisherProxy->topicType == "std_msgs/msg/Float32")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<std_msgs::msg::Float32> > >(publisherProxy->publisher);
        std_msgs::msg::Float32 msg;
        read__std_msgs__msg__Float32(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
    else if(publisherProxy->topicType == "std_msgs/msg/Float64")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<std_msgs::msg::Float64> > >(publisherProxy->publisher);
        std_msgs::msg::Float64 msg;
        read__std_msgs__msg__Float64(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
    else if(publisherProxy->topicType == "std_msgs/msg/Header")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<std_msgs::msg::Header> > >(publisherProxy->publisher);
        std_msgs::msg::Header msg;
        read__std_msgs__msg__Header(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
    else if(publisherProxy->topicType == "std_msgs/msg/Int16")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<std_msgs::msg::Int16> > >(publisherProxy->publisher);
        std_msgs::msg::Int16 msg;
        read__std_msgs__msg__Int16(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
    else if(publisherProxy->topicType == "std_msgs/msg/Int32")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<std_msgs::msg::Int32> > >(publisherProxy->publisher);
        std_msgs::msg::Int32 msg;
        read__std_msgs__msg__Int32(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
    else if(publisherProxy->topicType == "std_msgs/msg/Int64")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<std_msgs::msg::Int64> > >(publisherProxy->publisher);
        std_msgs::msg::Int64 msg;
        read__std_msgs__msg__Int64(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
    else if(publisherProxy->topicType == "std_msgs/msg/Int8")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<std_msgs::msg::Int8> > >(publisherProxy->publisher);
        std_msgs::msg::Int8 msg;
        read__std_msgs__msg__Int8(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
    else if(publisherProxy->topicType == "std_msgs/msg/String")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<std_msgs::msg::String> > >(publisherProxy->publisher);
        std_msgs::msg::String msg;
        read__std_msgs__msg__String(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
    else if(publisherProxy->topicType == "std_msgs/msg/UInt16")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<std_msgs::msg::UInt16> > >(publisherProxy->publisher);
        std_msgs::msg::UInt16 msg;
        read__std_msgs__msg__UInt16(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
    else if(publisherProxy->topicType == "std_msgs/msg/UInt32")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<std_msgs::msg::UInt32> > >(publisherProxy->publisher);
        std_msgs::msg::UInt32 msg;
        read__std_msgs__msg__UInt32(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
    else if(publisherProxy->topicType == "std_msgs/msg/UInt64")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<std_msgs::msg::UInt64> > >(publisherProxy->publisher);
        std_msgs::msg::UInt64 msg;
        read__std_msgs__msg__UInt64(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
    else if(publisherProxy->topicType == "std_msgs/msg/UInt8")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<std_msgs::msg::UInt8> > >(publisherProxy->publisher);
        std_msgs::msg::UInt8 msg;
        read__std_msgs__msg__UInt8(in->_.stackID, &msg, &(publisherProxy->rd_opt));
        pub->publish(msg);
    }
