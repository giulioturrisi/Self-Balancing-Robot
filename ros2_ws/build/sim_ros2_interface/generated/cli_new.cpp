    else if(in->serviceType == "example_interfaces/srv/AddTwoInts")
    {
        clientProxy->client = node->create_client<example_interfaces::srv::AddTwoInts>(in->serviceName);
    }
    else if(in->serviceType == "std_srvs/srv/Empty")
    {
        clientProxy->client = node->create_client<std_srvs::srv::Empty>(in->serviceName);
    }
    else if(in->serviceType == "std_srvs/srv/SetBool")
    {
        clientProxy->client = node->create_client<std_srvs::srv::SetBool>(in->serviceName);
    }
    else if(in->serviceType == "std_srvs/srv/Trigger")
    {
        clientProxy->client = node->create_client<std_srvs::srv::Trigger>(in->serviceName);
    }
