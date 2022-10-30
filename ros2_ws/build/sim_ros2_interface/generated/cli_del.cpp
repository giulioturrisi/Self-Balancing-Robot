    else if(clientProxy->serviceType == "example_interfaces/srv/AddTwoInts")
    {
        auto cli = boost::any_cast< std::shared_ptr< rclcpp::Client<example_interfaces::srv::AddTwoInts> > >(clientProxy->client);
        cli = nullptr;
    }
    else if(clientProxy->serviceType == "std_srvs/srv/Empty")
    {
        auto cli = boost::any_cast< std::shared_ptr< rclcpp::Client<std_srvs::srv::Empty> > >(clientProxy->client);
        cli = nullptr;
    }
    else if(clientProxy->serviceType == "std_srvs/srv/SetBool")
    {
        auto cli = boost::any_cast< std::shared_ptr< rclcpp::Client<std_srvs::srv::SetBool> > >(clientProxy->client);
        cli = nullptr;
    }
    else if(clientProxy->serviceType == "std_srvs/srv/Trigger")
    {
        auto cli = boost::any_cast< std::shared_ptr< rclcpp::Client<std_srvs::srv::Trigger> > >(clientProxy->client);
        cli = nullptr;
    }
