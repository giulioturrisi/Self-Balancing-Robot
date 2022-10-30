    else if(in->type == "builtin_interfaces/msg/Duration")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "builtin_interfaces/msg/Time")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "example_interfaces/action/FibonacciGoal")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "example_interfaces/action/FibonacciFeedback")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "example_interfaces/action/FibonacciResult")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "example_interfaces/srv/AddTwoIntsRequest")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "example_interfaces/srv/AddTwoIntsResponse")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "geometry_msgs/msg/Quaternion")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "geometry_msgs/msg/Transform")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "geometry_msgs/msg/TransformStamped")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "geometry_msgs/msg/Vector3")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "sensor_msgs/msg/Image")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "sensor_msgs/msg/PointCloud2")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "sensor_msgs/msg/PointField")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
        sim::pushStringOntoStack(in->_.stackID, "INT8", 0);
        write__uint8(1, in->_.stackID, &wo);
        sim::insertDataIntoStackTable(in->_.stackID);
        sim::pushStringOntoStack(in->_.stackID, "UINT8", 0);
        write__uint8(2, in->_.stackID, &wo);
        sim::insertDataIntoStackTable(in->_.stackID);
        sim::pushStringOntoStack(in->_.stackID, "INT16", 0);
        write__uint8(3, in->_.stackID, &wo);
        sim::insertDataIntoStackTable(in->_.stackID);
        sim::pushStringOntoStack(in->_.stackID, "UINT16", 0);
        write__uint8(4, in->_.stackID, &wo);
        sim::insertDataIntoStackTable(in->_.stackID);
        sim::pushStringOntoStack(in->_.stackID, "INT32", 0);
        write__uint8(5, in->_.stackID, &wo);
        sim::insertDataIntoStackTable(in->_.stackID);
        sim::pushStringOntoStack(in->_.stackID, "UINT32", 0);
        write__uint8(6, in->_.stackID, &wo);
        sim::insertDataIntoStackTable(in->_.stackID);
        sim::pushStringOntoStack(in->_.stackID, "FLOAT32", 0);
        write__uint8(7, in->_.stackID, &wo);
        sim::insertDataIntoStackTable(in->_.stackID);
        sim::pushStringOntoStack(in->_.stackID, "FLOAT64", 0);
        write__uint8(8, in->_.stackID, &wo);
        sim::insertDataIntoStackTable(in->_.stackID);
    }
    else if(in->type == "std_msgs/msg/Bool")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "std_msgs/msg/Byte")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "std_msgs/msg/ColorRGBA")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "std_msgs/msg/Empty")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "std_msgs/msg/Float32")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "std_msgs/msg/Float64")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "std_msgs/msg/Header")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "std_msgs/msg/Int16")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "std_msgs/msg/Int32")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "std_msgs/msg/Int64")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "std_msgs/msg/Int8")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "std_msgs/msg/String")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "std_msgs/msg/UInt16")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "std_msgs/msg/UInt32")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "std_msgs/msg/UInt64")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "std_msgs/msg/UInt8")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "std_srvs/srv/EmptyRequest")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "std_srvs/srv/EmptyResponse")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "std_srvs/srv/SetBoolRequest")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "std_srvs/srv/SetBoolResponse")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "std_srvs/srv/TriggerRequest")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
    else if(in->type == "std_srvs/srv/TriggerResponse")
    {
        ROS2WriteOptions wo;
        sim::pushTableOntoStack(in->_.stackID);
    }
