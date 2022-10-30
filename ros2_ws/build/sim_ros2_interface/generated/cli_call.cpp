    else if(clientProxy->serviceType == "example_interfaces/srv/AddTwoInts")
    {
        auto cli = boost::any_cast< std::shared_ptr< rclcpp::Client<example_interfaces::srv::AddTwoInts> > >(clientProxy->client);
        auto req = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        read__example_interfaces__srv__AddTwoInts__Request(in->_.stackID, req.get(), &(clientProxy->rd_opt));
        auto result = cli->async_send_request(req);
        if(rclcpp::spin_until_future_complete(node, result) ==
                rclcpp::FutureReturnCode::SUCCESS)
        {
            auto resp = result.get();
            write__example_interfaces__srv__AddTwoInts__Response(*resp, in->_.stackID, &(clientProxy->wr_opt));
        }
        else
        {
            throw sim::exception("failed to call service example_interfaces/srv/AddTwoInts");
        }
    }
    else if(clientProxy->serviceType == "std_srvs/srv/Empty")
    {
        auto cli = boost::any_cast< std::shared_ptr< rclcpp::Client<std_srvs::srv::Empty> > >(clientProxy->client);
        auto req = std::make_shared<std_srvs::srv::Empty::Request>();
        read__std_srvs__srv__Empty__Request(in->_.stackID, req.get(), &(clientProxy->rd_opt));
        auto result = cli->async_send_request(req);
        if(rclcpp::spin_until_future_complete(node, result) ==
                rclcpp::FutureReturnCode::SUCCESS)
        {
            auto resp = result.get();
            write__std_srvs__srv__Empty__Response(*resp, in->_.stackID, &(clientProxy->wr_opt));
        }
        else
        {
            throw sim::exception("failed to call service std_srvs/srv/Empty");
        }
    }
    else if(clientProxy->serviceType == "std_srvs/srv/SetBool")
    {
        auto cli = boost::any_cast< std::shared_ptr< rclcpp::Client<std_srvs::srv::SetBool> > >(clientProxy->client);
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        read__std_srvs__srv__SetBool__Request(in->_.stackID, req.get(), &(clientProxy->rd_opt));
        auto result = cli->async_send_request(req);
        if(rclcpp::spin_until_future_complete(node, result) ==
                rclcpp::FutureReturnCode::SUCCESS)
        {
            auto resp = result.get();
            write__std_srvs__srv__SetBool__Response(*resp, in->_.stackID, &(clientProxy->wr_opt));
        }
        else
        {
            throw sim::exception("failed to call service std_srvs/srv/SetBool");
        }
    }
    else if(clientProxy->serviceType == "std_srvs/srv/Trigger")
    {
        auto cli = boost::any_cast< std::shared_ptr< rclcpp::Client<std_srvs::srv::Trigger> > >(clientProxy->client);
        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        read__std_srvs__srv__Trigger__Request(in->_.stackID, req.get(), &(clientProxy->rd_opt));
        auto result = cli->async_send_request(req);
        if(rclcpp::spin_until_future_complete(node, result) ==
                rclcpp::FutureReturnCode::SUCCESS)
        {
            auto resp = result.get();
            write__std_srvs__srv__Trigger__Response(*resp, in->_.stackID, &(clientProxy->wr_opt));
        }
        else
        {
            throw sim::exception("failed to call service std_srvs/srv/Trigger");
        }
    }
