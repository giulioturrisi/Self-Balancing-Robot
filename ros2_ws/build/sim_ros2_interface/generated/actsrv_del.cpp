    else if(actionServerProxy->actionType == "example_interfaces/action/Fibonacci")
    {
        auto srv = boost::any_cast< std::shared_ptr< rclcpp_action::Server<example_interfaces::action::Fibonacci> > >(actionServerProxy->action_server);
        srv = nullptr;
    }
