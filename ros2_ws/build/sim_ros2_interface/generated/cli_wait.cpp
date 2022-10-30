    else if(clientProxy->serviceType == "example_interfaces/srv/AddTwoInts")
    {
        auto cli = boost::any_cast< std::shared_ptr< rclcpp::Client<example_interfaces::srv::AddTwoInts> > >(clientProxy->client);
        long timeout_ms = 1000 * in->timeout;
        out->result = cli->wait_for_service(std::chrono::milliseconds(timeout_ms));
    }
    else if(clientProxy->serviceType == "std_srvs/srv/Empty")
    {
        auto cli = boost::any_cast< std::shared_ptr< rclcpp::Client<std_srvs::srv::Empty> > >(clientProxy->client);
        long timeout_ms = 1000 * in->timeout;
        out->result = cli->wait_for_service(std::chrono::milliseconds(timeout_ms));
    }
    else if(clientProxy->serviceType == "std_srvs/srv/SetBool")
    {
        auto cli = boost::any_cast< std::shared_ptr< rclcpp::Client<std_srvs::srv::SetBool> > >(clientProxy->client);
        long timeout_ms = 1000 * in->timeout;
        out->result = cli->wait_for_service(std::chrono::milliseconds(timeout_ms));
    }
    else if(clientProxy->serviceType == "std_srvs/srv/Trigger")
    {
        auto cli = boost::any_cast< std::shared_ptr< rclcpp::Client<std_srvs::srv::Trigger> > >(clientProxy->client);
        long timeout_ms = 1000 * in->timeout;
        out->result = cli->wait_for_service(std::chrono::milliseconds(timeout_ms));
    }
