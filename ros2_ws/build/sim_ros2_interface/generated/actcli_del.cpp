    else if(actionClientProxy->actionType == "example_interfaces/action/Fibonacci")
    {
        auto cli = boost::any_cast< std::shared_ptr< rclcpp_action::Client<example_interfaces::action::Fibonacci> > >(actionClientProxy->action_client);
        cli = nullptr;
    }
