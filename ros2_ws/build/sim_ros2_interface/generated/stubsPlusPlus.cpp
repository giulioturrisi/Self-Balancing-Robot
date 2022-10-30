
// include this file after calling the SIM_PLUGIN(...) macro

void createSubscription(SScriptCallBack *p, const char *cmd, createSubscription_in *in, createSubscription_out *out)
{
    sim::plugin->createSubscription(in, out);
}
void shutdownSubscription(SScriptCallBack *p, const char *cmd, shutdownSubscription_in *in, shutdownSubscription_out *out)
{
    sim::plugin->shutdownSubscription(in, out);
}
void subscriptionTreatUInt8ArrayAsString(SScriptCallBack *p, const char *cmd, subscriptionTreatUInt8ArrayAsString_in *in, subscriptionTreatUInt8ArrayAsString_out *out)
{
    sim::plugin->subscriptionTreatUInt8ArrayAsString(in, out);
}
void createPublisher(SScriptCallBack *p, const char *cmd, createPublisher_in *in, createPublisher_out *out)
{
    sim::plugin->createPublisher(in, out);
}
void shutdownPublisher(SScriptCallBack *p, const char *cmd, shutdownPublisher_in *in, shutdownPublisher_out *out)
{
    sim::plugin->shutdownPublisher(in, out);
}
void publisherTreatUInt8ArrayAsString(SScriptCallBack *p, const char *cmd, publisherTreatUInt8ArrayAsString_in *in, publisherTreatUInt8ArrayAsString_out *out)
{
    sim::plugin->publisherTreatUInt8ArrayAsString(in, out);
}
void publish(SScriptCallBack *p, const char *cmd, publish_in *in, publish_out *out)
{
    sim::plugin->publish(in, out);
}
void createClient(SScriptCallBack *p, const char *cmd, createClient_in *in, createClient_out *out)
{
    sim::plugin->createClient(in, out);
}
void shutdownClient(SScriptCallBack *p, const char *cmd, shutdownClient_in *in, shutdownClient_out *out)
{
    sim::plugin->shutdownClient(in, out);
}
void clientTreatUInt8ArrayAsString(SScriptCallBack *p, const char *cmd, clientTreatUInt8ArrayAsString_in *in, clientTreatUInt8ArrayAsString_out *out)
{
    sim::plugin->clientTreatUInt8ArrayAsString(in, out);
}
void waitForService(SScriptCallBack *p, const char *cmd, waitForService_in *in, waitForService_out *out)
{
    sim::plugin->waitForService(in, out);
}
void call(SScriptCallBack *p, const char *cmd, call_in *in, call_out *out)
{
    sim::plugin->call(in, out);
}
void createService(SScriptCallBack *p, const char *cmd, createService_in *in, createService_out *out)
{
    sim::plugin->createService(in, out);
}
void shutdownService(SScriptCallBack *p, const char *cmd, shutdownService_in *in, shutdownService_out *out)
{
    sim::plugin->shutdownService(in, out);
}
void serviceTreatUInt8ArrayAsString(SScriptCallBack *p, const char *cmd, serviceTreatUInt8ArrayAsString_in *in, serviceTreatUInt8ArrayAsString_out *out)
{
    sim::plugin->serviceTreatUInt8ArrayAsString(in, out);
}
void createActionClient(SScriptCallBack *p, const char *cmd, createActionClient_in *in, createActionClient_out *out)
{
    sim::plugin->createActionClient(in, out);
}
void shutdownActionClient(SScriptCallBack *p, const char *cmd, shutdownActionClient_in *in, shutdownActionClient_out *out)
{
    sim::plugin->shutdownActionClient(in, out);
}
void actionClientTreatUInt8ArrayAsString(SScriptCallBack *p, const char *cmd, actionClientTreatUInt8ArrayAsString_in *in, actionClientTreatUInt8ArrayAsString_out *out)
{
    sim::plugin->actionClientTreatUInt8ArrayAsString(in, out);
}
void sendGoal(SScriptCallBack *p, const char *cmd, sendGoal_in *in, sendGoal_out *out)
{
    sim::plugin->sendGoal(in, out);
}
void cancelLastGoal(SScriptCallBack *p, const char *cmd, cancelLastGoal_in *in, cancelLastGoal_out *out)
{
    sim::plugin->cancelLastGoal(in, out);
}
void createActionServer(SScriptCallBack *p, const char *cmd, createActionServer_in *in, createActionServer_out *out)
{
    sim::plugin->createActionServer(in, out);
}
void shutdownActionServer(SScriptCallBack *p, const char *cmd, shutdownActionServer_in *in, shutdownActionServer_out *out)
{
    sim::plugin->shutdownActionServer(in, out);
}
void actionServerTreatUInt8ArrayAsString(SScriptCallBack *p, const char *cmd, actionServerTreatUInt8ArrayAsString_in *in, actionServerTreatUInt8ArrayAsString_out *out)
{
    sim::plugin->actionServerTreatUInt8ArrayAsString(in, out);
}
void actionServerPublishFeedback(SScriptCallBack *p, const char *cmd, actionServerPublishFeedback_in *in, actionServerPublishFeedback_out *out)
{
    sim::plugin->actionServerPublishFeedback(in, out);
}
void actionServerActionAbort(SScriptCallBack *p, const char *cmd, actionServerActionAbort_in *in, actionServerActionAbort_out *out)
{
    sim::plugin->actionServerActionAbort(in, out);
}
void actionServerActionSucceed(SScriptCallBack *p, const char *cmd, actionServerActionSucceed_in *in, actionServerActionSucceed_out *out)
{
    sim::plugin->actionServerActionSucceed(in, out);
}
void actionServerActionCanceled(SScriptCallBack *p, const char *cmd, actionServerActionCanceled_in *in, actionServerActionCanceled_out *out)
{
    sim::plugin->actionServerActionCanceled(in, out);
}
void actionServerActionExecute(SScriptCallBack *p, const char *cmd, actionServerActionExecute_in *in, actionServerActionExecute_out *out)
{
    sim::plugin->actionServerActionExecute(in, out);
}
void actionServerActionIsCanceling(SScriptCallBack *p, const char *cmd, actionServerActionIsCanceling_in *in, actionServerActionIsCanceling_out *out)
{
    sim::plugin->actionServerActionIsCanceling(in, out);
}
void actionServerActionIsActive(SScriptCallBack *p, const char *cmd, actionServerActionIsActive_in *in, actionServerActionIsActive_out *out)
{
    sim::plugin->actionServerActionIsActive(in, out);
}
void actionServerActionIsExecuting(SScriptCallBack *p, const char *cmd, actionServerActionIsExecuting_in *in, actionServerActionIsExecuting_out *out)
{
    sim::plugin->actionServerActionIsExecuting(in, out);
}
void sendTransform(SScriptCallBack *p, const char *cmd, sendTransform_in *in, sendTransform_out *out)
{
    sim::plugin->sendTransform(in, out);
}
void sendTransforms(SScriptCallBack *p, const char *cmd, sendTransforms_in *in, sendTransforms_out *out)
{
    sim::plugin->sendTransforms(in, out);
}
void imageTransportCreateSubscription(SScriptCallBack *p, const char *cmd, imageTransportCreateSubscription_in *in, imageTransportCreateSubscription_out *out)
{
    sim::plugin->imageTransportCreateSubscription(in, out);
}
void imageTransportShutdownSubscription(SScriptCallBack *p, const char *cmd, imageTransportShutdownSubscription_in *in, imageTransportShutdownSubscription_out *out)
{
    sim::plugin->imageTransportShutdownSubscription(in, out);
}
void imageTransportCreatePublisher(SScriptCallBack *p, const char *cmd, imageTransportCreatePublisher_in *in, imageTransportCreatePublisher_out *out)
{
    sim::plugin->imageTransportCreatePublisher(in, out);
}
void imageTransportShutdownPublisher(SScriptCallBack *p, const char *cmd, imageTransportShutdownPublisher_in *in, imageTransportShutdownPublisher_out *out)
{
    sim::plugin->imageTransportShutdownPublisher(in, out);
}
void imageTransportPublish(SScriptCallBack *p, const char *cmd, imageTransportPublish_in *in, imageTransportPublish_out *out)
{
    sim::plugin->imageTransportPublish(in, out);
}
void getTime(SScriptCallBack *p, const char *cmd, getTime_in *in, getTime_out *out)
{
    sim::plugin->getTime(in, out);
}
void getParamString(SScriptCallBack *p, const char *cmd, getParamString_in *in, getParamString_out *out)
{
    sim::plugin->getParamString(in, out);
}
void getParamInt(SScriptCallBack *p, const char *cmd, getParamInt_in *in, getParamInt_out *out)
{
    sim::plugin->getParamInt(in, out);
}
void getParamDouble(SScriptCallBack *p, const char *cmd, getParamDouble_in *in, getParamDouble_out *out)
{
    sim::plugin->getParamDouble(in, out);
}
void getParamBool(SScriptCallBack *p, const char *cmd, getParamBool_in *in, getParamBool_out *out)
{
    sim::plugin->getParamBool(in, out);
}
void setParamString(SScriptCallBack *p, const char *cmd, setParamString_in *in, setParamString_out *out)
{
    sim::plugin->setParamString(in, out);
}
void setParamInt(SScriptCallBack *p, const char *cmd, setParamInt_in *in, setParamInt_out *out)
{
    sim::plugin->setParamInt(in, out);
}
void setParamDouble(SScriptCallBack *p, const char *cmd, setParamDouble_in *in, setParamDouble_out *out)
{
    sim::plugin->setParamDouble(in, out);
}
void setParamBool(SScriptCallBack *p, const char *cmd, setParamBool_in *in, setParamBool_out *out)
{
    sim::plugin->setParamBool(in, out);
}
void hasParam(SScriptCallBack *p, const char *cmd, hasParam_in *in, hasParam_out *out)
{
    sim::plugin->hasParam(in, out);
}
void deleteParam(SScriptCallBack *p, const char *cmd, deleteParam_in *in, deleteParam_out *out)
{
    sim::plugin->deleteParam(in, out);
}
void createInterface(SScriptCallBack *p, const char *cmd, createInterface_in *in, createInterface_out *out)
{
    sim::plugin->createInterface(in, out);
}
void getInterfaceConstants(SScriptCallBack *p, const char *cmd, getInterfaceConstants_in *in, getInterfaceConstants_out *out)
{
    sim::plugin->getInterfaceConstants(in, out);
}
void supportedInterfaces(SScriptCallBack *p, const char *cmd, supportedInterfaces_in *in, supportedInterfaces_out *out)
{
    sim::plugin->supportedInterfaces(in, out);
}

