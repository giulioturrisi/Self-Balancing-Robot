    else if(actionServerProxy->actionType == "example_interfaces/action/Fibonacci")
    {
        auto gh = getGoalHandle<example_interfaces::action::Fibonacci>(actionServerProxy, in->goalUUID);
        auto result = std::make_shared<example_interfaces::action::Fibonacci::Result>();
        read__example_interfaces__action__Fibonacci__Result(in->_.stackID, result.get(), &(actionServerProxy->rd_opt));
        gh->abort(result);
    }
