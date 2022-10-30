    else if(actionClientProxy->actionType == "example_interfaces/action/Fibonacci")
    {
        auto cli = boost::any_cast< std::shared_ptr< rclcpp_action::Client<example_interfaces::action::Fibonacci> > >(actionClientProxy->action_client);
        if(!cli->wait_for_action_server(std::chrono::seconds(5)))
            throw sim::exception("action server not available after wait");
        example_interfaces::action::Fibonacci::Goal goal_msg;
        read__example_interfaces__action__Fibonacci__Goal(in->_.stackID, &goal_msg, &(actionClientProxy->rd_opt));
        auto send_goal_options = rclcpp_action::Client<example_interfaces::action::Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback = [=] (std::shared_ptr< rclcpp_action::ClientGoalHandle<example_interfaces::action::Fibonacci> > handle) -> void
        {
            actionGoalResponseCallback_in in1;
            actionGoalResponseCallback_out out1;
            auto goal_handle = handle.get();
            in1.goalID = goal_handle ? goalUUIDtoString(goal_handle->get_goal_id()) : "";
            in1.accepted = !!goal_handle;
            actionGoalResponseCallback(actionClientProxy->goalResponseCallback.scriptId, actionClientProxy->goalResponseCallback.name.c_str(), &in1, &out1);
        };
        send_goal_options.feedback_callback = [=] (rclcpp_action::ClientGoalHandle<example_interfaces::action::Fibonacci>::SharedPtr goal_handle, const std::shared_ptr<const example_interfaces::action::Fibonacci::Feedback> feedback) -> void
        {
            ros_action_callback__example_interfaces__action__Fibonacci__Feedback(actionClientProxy->feedbackCallback.scriptId, actionClientProxy->feedbackCallback.name.c_str(), goal_handle->get_goal_id(), feedback.get(), actionClientProxy);
        };
        send_goal_options.result_callback = [=] (const rclcpp_action::ClientGoalHandle<example_interfaces::action::Fibonacci>::WrappedResult &result) -> void
        {
            int lua_code;
            switch(result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                lua_code = sim_ros2_action_result_code_succeeded;
                break;
            case rclcpp_action::ResultCode::ABORTED:
                lua_code = sim_ros2_action_result_code_aborted;
                break;
            case rclcpp_action::ResultCode::CANCELED:
                lua_code = sim_ros2_action_result_code_canceled;
                break;
            default:
                lua_code = sim_ros2_action_result_code_unknown;
                break;
            }
            ros_action_callback__example_interfaces__action__Fibonacci__Result(actionClientProxy->resultCallback.scriptId, actionClientProxy->resultCallback.name.c_str(), result.goal_id, lua_code, result.result, actionClientProxy);
        };
        auto goal_handle_future = cli->async_send_goal(goal_msg, send_goal_options);
        out->success = rclcpp::spin_until_future_complete(node, goal_handle_future) == rclcpp::FutureReturnCode::SUCCESS;
        rclcpp_action::ClientGoalHandle<example_interfaces::action::Fibonacci>::SharedPtr goal_handle = goal_handle_future.get();
        actionClientProxy->last_goal_handle = goal_handle;
    }
