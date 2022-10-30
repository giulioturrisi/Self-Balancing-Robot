    else if(actionServerProxy->actionType == "example_interfaces/action/Fibonacci")
    {
        auto gh = getGoalHandle<example_interfaces::action::Fibonacci>(actionServerProxy, in->goalUUID);
        out->result = gh->is_executing();
    }
