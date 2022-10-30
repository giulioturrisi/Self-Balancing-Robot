    else if(serviceProxy->serviceType == "example_interfaces/srv/AddTwoInts")
    {
        auto srv = boost::any_cast< std::shared_ptr< rclcpp::Service<example_interfaces::srv::AddTwoInts> > >(serviceProxy->service);
        srv = nullptr;
    }
    else if(serviceProxy->serviceType == "std_srvs/srv/Empty")
    {
        auto srv = boost::any_cast< std::shared_ptr< rclcpp::Service<std_srvs::srv::Empty> > >(serviceProxy->service);
        srv = nullptr;
    }
    else if(serviceProxy->serviceType == "std_srvs/srv/SetBool")
    {
        auto srv = boost::any_cast< std::shared_ptr< rclcpp::Service<std_srvs::srv::SetBool> > >(serviceProxy->service);
        srv = nullptr;
    }
    else if(serviceProxy->serviceType == "std_srvs/srv/Trigger")
    {
        auto srv = boost::any_cast< std::shared_ptr< rclcpp::Service<std_srvs::srv::Trigger> > >(serviceProxy->service);
        srv = nullptr;
    }
