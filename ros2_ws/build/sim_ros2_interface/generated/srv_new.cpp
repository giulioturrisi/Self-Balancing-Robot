    else if(in->serviceType == "example_interfaces/srv/AddTwoInts")
    {
        auto cb = [=](const std::shared_ptr<rmw_request_id_t> request_header, const example_interfaces::srv::AddTwoInts::Request::SharedPtr req, example_interfaces::srv::AddTwoInts::Response::SharedPtr res) { ros_srv_callback__example_interfaces__srv__AddTwoInts(request_header, req, res, serviceProxy); };
        serviceProxy->service = node->create_service<example_interfaces::srv::AddTwoInts>(in->serviceName, cb);
    }
    else if(in->serviceType == "std_srvs/srv/Empty")
    {
        auto cb = [=](const std::shared_ptr<rmw_request_id_t> request_header, const std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res) { ros_srv_callback__std_srvs__srv__Empty(request_header, req, res, serviceProxy); };
        serviceProxy->service = node->create_service<std_srvs::srv::Empty>(in->serviceName, cb);
    }
    else if(in->serviceType == "std_srvs/srv/SetBool")
    {
        auto cb = [=](const std::shared_ptr<rmw_request_id_t> request_header, const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res) { ros_srv_callback__std_srvs__srv__SetBool(request_header, req, res, serviceProxy); };
        serviceProxy->service = node->create_service<std_srvs::srv::SetBool>(in->serviceName, cb);
    }
    else if(in->serviceType == "std_srvs/srv/Trigger")
    {
        auto cb = [=](const std::shared_ptr<rmw_request_id_t> request_header, const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res) { ros_srv_callback__std_srvs__srv__Trigger(request_header, req, res, serviceProxy); };
        serviceProxy->service = node->create_service<std_srvs::srv::Trigger>(in->serviceName, cb);
    }
