    else if(in->actionType == "example_interfaces/action/Fibonacci")
    {
        auto handle_goal_cb = [=] (const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const example_interfaces::action::Fibonacci::Goal> goal) -> rclcpp_action::GoalResponse
        {
            sim::addLog(sim_verbosity_trace, "Calling handle goal callback...");
            const auto &cb = actionServerProxy->handleGoalCallback;
            return ros_action_callback__handle_goal__example_interfaces__action__Fibonacci__Goal(cb.scriptId, cb.name.c_str(), uuid, goal.get(), actionServerProxy);
        };
        auto handle_cancel_cb = [=] (const std::shared_ptr< rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci> > goal_handle) -> rclcpp_action::CancelResponse
        {
            sim::addLog(sim_verbosity_trace, "Calling handle cancel callback...");
            const auto &cb = actionServerProxy->handleCancelCallback;
            return ros_action_callback__handle_cancel__example_interfaces__action__Fibonacci__Goal(cb.scriptId, cb.name.c_str(), goal_handle->get_goal_id(), goal_handle->get_goal().get(), actionServerProxy);
        };
        auto handle_accepted_cb = [=] (const std::shared_ptr< rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci> > goal_handle)
        {
            sim::addLog(sim_verbosity_trace, "Calling handle accepted callback...");
            actionServerProxy->goalHandles[goal_handle->get_goal_id()] = goal_handle;
            const auto &cb = actionServerProxy->handleAcceptedCallback;
            ros_action_callback__handle_accepted__example_interfaces__action__Fibonacci__Goal(cb.scriptId, cb.name.c_str(), goal_handle->get_goal_id(), goal_handle->get_goal().get(), actionServerProxy);
        };
        actionServerProxy->action_server = rclcpp_action::create_server<example_interfaces::action::Fibonacci>(node, in->actionName, handle_goal_cb, handle_cancel_cb, handle_accepted_cb);
    }
