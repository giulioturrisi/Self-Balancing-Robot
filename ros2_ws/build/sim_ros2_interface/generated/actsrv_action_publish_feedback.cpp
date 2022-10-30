    else if(actionServerProxy->actionType == "example_interfaces/action/Fibonacci")
    {
        auto gh = getGoalHandle<example_interfaces::action::Fibonacci>(actionServerProxy, in->goalUUID);
        auto feedback = std::make_shared<example_interfaces::action::Fibonacci::Feedback>();
        read__example_interfaces__action__Fibonacci__Feedback(in->_.stackID, feedback.get(), &(actionServerProxy->rd_opt));
        gh->publish_feedback(feedback);
    }
