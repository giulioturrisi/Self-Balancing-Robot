    else if(actionClientProxy->actionType == "example_interfaces/action/Fibonacci")
    {
        auto cli = boost::any_cast< std::shared_ptr< rclcpp_action::Client<example_interfaces::action::Fibonacci> > >(actionClientProxy->action_client);
        auto gh = boost::any_cast< rclcpp_action::ClientGoalHandle<example_interfaces::action::Fibonacci>::SharedPtr >(actionClientProxy->last_goal_handle);
        auto cancel_result_future = cli->async_cancel_goal(gh);
        if(rclcpp::spin_until_future_complete(node, cancel_result_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            out->success = false;
        }
        else
        {
            out->success = true;
        }
    }
