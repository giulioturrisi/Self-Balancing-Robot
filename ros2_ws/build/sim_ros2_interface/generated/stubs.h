#ifndef STUBS_H__INCLUDED
#define STUBS_H__INCLUDED

#include <simPlusPlus/Lib.h>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
#include <boost/optional.hpp>

class FuncTracer
{
    int l_;
    std::string f_;
public:
    FuncTracer(const std::string &f, int l = sim_verbosity_trace);
    ~FuncTracer();
};

#ifndef __FUNC__
#ifdef __PRETTY_FUNCTION__
#define __FUNC__ __PRETTY_FUNCTION__
#else
#define __FUNC__ __func__
#endif
#endif // __FUNC__

#define TRACE_FUNC FuncTracer __funcTracer__##__LINE__((boost::format("%s:%d:%s:") % __FILE__ % __LINE__ % __FUNC__).str())

#ifdef QT_COMPIL
#include <QThread>

extern Qt::HANDLE UI_THREAD;
extern Qt::HANDLE SIM_THREAD;

std::string threadNickname();
void uiThread();
void simThread();

#define ASSERT_THREAD(ID) \
    if(UI_THREAD == NULL) {\
        sim::addLog(sim_verbosity_debug, "warning: cannot check ASSERT_THREAD(" #ID ") because global variable UI_THREAD is not set yet.");\
    } else if(strcmp(#ID, "UI") == 0) {\
        if(QThread::currentThreadId() != UI_THREAD) {\
            sim::addLog(sim_verbosity_errors, "%s:%d %s should be called from UI thread", __FILE__, __LINE__, __FUNC__);\
            exit(1);\
        }\
    } else if(strcmp(#ID, "!UI") == 0) {\
        if(QThread::currentThreadId() == UI_THREAD) {\
            sim::addLog(sim_verbosity_errors, "%s:%d %s should NOT be called from UI thread", __FILE__, __LINE__, __FUNC__);\
            exit(1);\
        }\
    } else {\
        sim::addLog(sim_verbosity_debug, "warning: cannot check ASSERT_THREAD(" #ID "). Can check only UI and !UI.");\
    }
#endif // QT_COMPIL

template<typename T>
struct Grid
{
    std::vector<int> dims;
    std::vector<T> data;
};

struct ReadOptions
{
    std::vector<size_t> minSize;
    std::vector<size_t> maxSize;

    ReadOptions& setBounds(size_t dim, size_t minSize_, size_t maxSize_)
    {
        while(minSize.size() <= dim) minSize.push_back(0);
        while(maxSize.size() <= dim) maxSize.push_back(std::numeric_limits<size_t>::max());
        minSize[dim] = minSize_;
        maxSize[dim] = maxSize_;
        return *this;
    }

    ReadOptions& setBounds(size_t dim, const std::string &s)
    {
        if(s == "*") return setBounds(dim, 0, -1);
        auto n = s.find("..");
        if(n == std::string::npos)
        {
            int f = std::stoi(s);
            return setBounds(dim, f, f);
        }
        else
        {
            std::string smin = s.substr(0, n);
            std::string smax = s.substr(n + 2);
            int min = std::stoi(smin);
            int max = smax == "*" ? -1 : std::stoi(smax);
            return setBounds(dim, min, max);
        }
    }

    ReadOptions& setBounds(const std::string &s)
    {
        if(s != "")
        {
            std::vector<std::string> ss;
            boost::split(ss, s, boost::is_any_of(", "));
            for(size_t dim = 0; dim < ss.size(); dim++)
                setBounds(dim, ss.at(dim));
        }
        return *this;
    }

    void validateTableSize(size_t sz) const
    {
        if(minSize.empty() || maxSize.empty()) return;
        if(minSize[0] == maxSize[0])
        {
            if(sz != minSize[0])
                throw sim::exception("must have exactly %d elements", minSize[0]);
        }
        else
        {
            if(sz < minSize[0])
                throw sim::exception("must have at least %d elements", minSize[0]);
            if(sz > maxSize[0])
                throw sim::exception("must have at most %d elements", maxSize[0]);
        }
    }

    void validateSize(size_t dim, size_t sz) const
    {
        if(dim >= minSize.size() || dim >= maxSize.size()) return;
        if(minSize[dim] == maxSize[dim])
        {
            if(sz != minSize[dim])
                throw sim::exception("dimension %d must have exactly %d elements", dim + 1, minSize[dim]);
        }
        else
        {
            if(sz < minSize[dim])
                throw sim::exception("dimension %d must have at least %d elements", dim + 1, minSize[dim]);
            if(sz > maxSize[dim])
                throw sim::exception("dimension %d must have at most %d elements", dim + 1, maxSize[dim]);
        }
    }

    template<typename T>
    void validateSize(const std::vector<T> &szs) const
    {
        size_t n = std::min(minSize.size(), maxSize.size());
        if(n && szs.size() != n)
            throw sim::exception("incorrect dimension count: %d (should be %d)", szs.size(), n);
        for(size_t dim = 0; dim < n; dim++)
            validateSize(dim, szs.at(dim));
    }
};

struct WriteOptions
{
    void *dummy{nullptr};
};

struct sim_ros2_time
{
    int sec;
    int nanosec;

    sim_ros2_time();
    sim_ros2_time(const int &sec_, const int &nanosec_) : sec(sec_), nanosec(nanosec_) {}
};

void readFromStack(int stack, bool *value, const ReadOptions &rdopt = {});
void readFromStack(int stack, int *value, const ReadOptions &rdopt = {});
void readFromStack(int stack, long *value, const ReadOptions &rdopt = {});
void readFromStack(int stack, float *value, const ReadOptions &rdopt = {});
void readFromStack(int stack, double *value, const ReadOptions &rdopt = {});
void readFromStack(int stack, std::string *value, const ReadOptions &rdopt = {});
void readFromStack(int stack, sim_ros2_time *value, const ReadOptions &rdopt = {});
void writeToStack(const bool &value, int stack, const WriteOptions &wropt = {});
void writeToStack(const int &value, int stack, const WriteOptions &wropt = {});
void writeToStack(const long &value, int stack, const WriteOptions &wropt = {});
void writeToStack(const float &value, int stack, const WriteOptions &wropt = {});
void writeToStack(const double &value, int stack, const WriteOptions &wropt = {});
void writeToStack(const std::string &value, int stack, const WriteOptions &wropt = {});
void writeToStack(const sim_ros2_time &value, int stack, const WriteOptions &wropt = {});

bool registerScriptStuff();

enum clock_type
{
    sim_ros2_clock_ros = 4872,
    sim_ros2_clock_system = 4873,
    sim_ros2_clock_steady = 4874,
};

const char* clock_type_string(clock_type x);

enum action_result_code
{
    sim_ros2_action_result_code_succeeded = 8564,
    sim_ros2_action_result_code_aborted = 8565,
    sim_ros2_action_result_code_canceled = 8566,
    sim_ros2_action_result_code_unknown = 8567,
};

const char* action_result_code_string(action_result_code x);

enum goal_response
{
    sim_ros2_goal_response_reject = 12856,
    sim_ros2_goal_response_accept_and_execute = 12857,
    sim_ros2_goal_response_accept_and_defer = 12858,
};

const char* goal_response_string(goal_response x);

enum cancel_response
{
    sim_ros2_cancel_response_reject = 16099,
    sim_ros2_cancel_response_accept = 16100,
};

const char* cancel_response_string(cancel_response x);

struct createSubscription_in
{
    SScriptCallBack _;
    std::string topicName;
    std::string topicType;
    std::string topicCallback;
    int queueSize;

    createSubscription_in();
};

struct createSubscription_out
{
    std::string subscriptionHandle;

    createSubscription_out();
};

void createSubscription(SScriptCallBack *p, createSubscription_in *in, createSubscription_out *out);
std::string createSubscription(SScriptCallBack *p, std::string topicName, std::string topicType, std::string topicCallback, int queueSize = 1);
void createSubscription(SScriptCallBack *p, createSubscription_out *out, std::string topicName, std::string topicType, std::string topicCallback, int queueSize = 1);
void createSubscription_callback(SScriptCallBack *p);

struct shutdownSubscription_in
{
    SScriptCallBack _;
    std::string subscriptionHandle;

    shutdownSubscription_in();
};

struct shutdownSubscription_out
{

    shutdownSubscription_out();
};

void shutdownSubscription(SScriptCallBack *p, shutdownSubscription_in *in, shutdownSubscription_out *out);
void shutdownSubscription(SScriptCallBack *p, std::string subscriptionHandle);
void shutdownSubscription(SScriptCallBack *p, shutdownSubscription_out *out, std::string subscriptionHandle);
void shutdownSubscription_callback(SScriptCallBack *p);

struct subscriptionTreatUInt8ArrayAsString_in
{
    SScriptCallBack _;
    std::string subscriptionHandle;

    subscriptionTreatUInt8ArrayAsString_in();
};

struct subscriptionTreatUInt8ArrayAsString_out
{

    subscriptionTreatUInt8ArrayAsString_out();
};

void subscriptionTreatUInt8ArrayAsString(SScriptCallBack *p, subscriptionTreatUInt8ArrayAsString_in *in, subscriptionTreatUInt8ArrayAsString_out *out);
void subscriptionTreatUInt8ArrayAsString(SScriptCallBack *p, std::string subscriptionHandle);
void subscriptionTreatUInt8ArrayAsString(SScriptCallBack *p, subscriptionTreatUInt8ArrayAsString_out *out, std::string subscriptionHandle);
void subscriptionTreatUInt8ArrayAsString_callback(SScriptCallBack *p);

struct createPublisher_in
{
    SScriptCallBack _;
    std::string topicName;
    std::string topicType;
    int queueSize;
    bool latch;

    createPublisher_in();
};

struct createPublisher_out
{
    std::string publisherHandle;

    createPublisher_out();
};

void createPublisher(SScriptCallBack *p, createPublisher_in *in, createPublisher_out *out);
std::string createPublisher(SScriptCallBack *p, std::string topicName, std::string topicType, int queueSize = 1, bool latch = false);
void createPublisher(SScriptCallBack *p, createPublisher_out *out, std::string topicName, std::string topicType, int queueSize = 1, bool latch = false);
void createPublisher_callback(SScriptCallBack *p);

struct shutdownPublisher_in
{
    SScriptCallBack _;
    std::string publisherHandle;

    shutdownPublisher_in();
};

struct shutdownPublisher_out
{

    shutdownPublisher_out();
};

void shutdownPublisher(SScriptCallBack *p, shutdownPublisher_in *in, shutdownPublisher_out *out);
void shutdownPublisher(SScriptCallBack *p, std::string publisherHandle);
void shutdownPublisher(SScriptCallBack *p, shutdownPublisher_out *out, std::string publisherHandle);
void shutdownPublisher_callback(SScriptCallBack *p);

struct publisherTreatUInt8ArrayAsString_in
{
    SScriptCallBack _;
    std::string publisherHandle;

    publisherTreatUInt8ArrayAsString_in();
};

struct publisherTreatUInt8ArrayAsString_out
{

    publisherTreatUInt8ArrayAsString_out();
};

void publisherTreatUInt8ArrayAsString(SScriptCallBack *p, publisherTreatUInt8ArrayAsString_in *in, publisherTreatUInt8ArrayAsString_out *out);
void publisherTreatUInt8ArrayAsString(SScriptCallBack *p, std::string publisherHandle);
void publisherTreatUInt8ArrayAsString(SScriptCallBack *p, publisherTreatUInt8ArrayAsString_out *out, std::string publisherHandle);
void publisherTreatUInt8ArrayAsString_callback(SScriptCallBack *p);

struct publish_in
{
    SScriptCallBack _;
    std::string publisherHandle;

    publish_in();
};

struct publish_out
{

    publish_out();
};

void publish(SScriptCallBack *p, publish_in *in, publish_out *out);
void publish(SScriptCallBack *p, std::string publisherHandle);
void publish(SScriptCallBack *p, publish_out *out, std::string publisherHandle);
void publish_callback(SScriptCallBack *p);

struct createClient_in
{
    SScriptCallBack _;
    std::string serviceName;
    std::string serviceType;

    createClient_in();
};

struct createClient_out
{
    std::string clientHandle;

    createClient_out();
};

void createClient(SScriptCallBack *p, createClient_in *in, createClient_out *out);
std::string createClient(SScriptCallBack *p, std::string serviceName, std::string serviceType);
void createClient(SScriptCallBack *p, createClient_out *out, std::string serviceName, std::string serviceType);
void createClient_callback(SScriptCallBack *p);

struct shutdownClient_in
{
    SScriptCallBack _;
    std::string clientHandle;

    shutdownClient_in();
};

struct shutdownClient_out
{

    shutdownClient_out();
};

void shutdownClient(SScriptCallBack *p, shutdownClient_in *in, shutdownClient_out *out);
void shutdownClient(SScriptCallBack *p, std::string clientHandle);
void shutdownClient(SScriptCallBack *p, shutdownClient_out *out, std::string clientHandle);
void shutdownClient_callback(SScriptCallBack *p);

struct clientTreatUInt8ArrayAsString_in
{
    SScriptCallBack _;
    std::string clientHandle;

    clientTreatUInt8ArrayAsString_in();
};

struct clientTreatUInt8ArrayAsString_out
{

    clientTreatUInt8ArrayAsString_out();
};

void clientTreatUInt8ArrayAsString(SScriptCallBack *p, clientTreatUInt8ArrayAsString_in *in, clientTreatUInt8ArrayAsString_out *out);
void clientTreatUInt8ArrayAsString(SScriptCallBack *p, std::string clientHandle);
void clientTreatUInt8ArrayAsString(SScriptCallBack *p, clientTreatUInt8ArrayAsString_out *out, std::string clientHandle);
void clientTreatUInt8ArrayAsString_callback(SScriptCallBack *p);

struct waitForService_in
{
    SScriptCallBack _;
    std::string clientHandle;
    float timeout;

    waitForService_in();
};

struct waitForService_out
{
    bool result;

    waitForService_out();
};

void waitForService(SScriptCallBack *p, waitForService_in *in, waitForService_out *out);
bool waitForService(SScriptCallBack *p, std::string clientHandle, float timeout);
void waitForService(SScriptCallBack *p, waitForService_out *out, std::string clientHandle, float timeout);
void waitForService_callback(SScriptCallBack *p);

struct call_in
{
    SScriptCallBack _;
    std::string clientHandle;

    call_in();
};

struct call_out
{

    call_out();
};

void call(SScriptCallBack *p, call_in *in, call_out *out);
void call(SScriptCallBack *p, std::string clientHandle);
void call(SScriptCallBack *p, call_out *out, std::string clientHandle);
void call_callback(SScriptCallBack *p);

struct createService_in
{
    SScriptCallBack _;
    std::string serviceName;
    std::string serviceType;
    std::string serviceCallback;

    createService_in();
};

struct createService_out
{
    std::string serviceHandle;

    createService_out();
};

void createService(SScriptCallBack *p, createService_in *in, createService_out *out);
std::string createService(SScriptCallBack *p, std::string serviceName, std::string serviceType, std::string serviceCallback);
void createService(SScriptCallBack *p, createService_out *out, std::string serviceName, std::string serviceType, std::string serviceCallback);
void createService_callback(SScriptCallBack *p);

struct shutdownService_in
{
    SScriptCallBack _;
    std::string serviceHandle;

    shutdownService_in();
};

struct shutdownService_out
{

    shutdownService_out();
};

void shutdownService(SScriptCallBack *p, shutdownService_in *in, shutdownService_out *out);
void shutdownService(SScriptCallBack *p, std::string serviceHandle);
void shutdownService(SScriptCallBack *p, shutdownService_out *out, std::string serviceHandle);
void shutdownService_callback(SScriptCallBack *p);

struct serviceTreatUInt8ArrayAsString_in
{
    SScriptCallBack _;
    std::string serviceHandle;

    serviceTreatUInt8ArrayAsString_in();
};

struct serviceTreatUInt8ArrayAsString_out
{

    serviceTreatUInt8ArrayAsString_out();
};

void serviceTreatUInt8ArrayAsString(SScriptCallBack *p, serviceTreatUInt8ArrayAsString_in *in, serviceTreatUInt8ArrayAsString_out *out);
void serviceTreatUInt8ArrayAsString(SScriptCallBack *p, std::string serviceHandle);
void serviceTreatUInt8ArrayAsString(SScriptCallBack *p, serviceTreatUInt8ArrayAsString_out *out, std::string serviceHandle);
void serviceTreatUInt8ArrayAsString_callback(SScriptCallBack *p);

struct createActionClient_in
{
    SScriptCallBack _;
    std::string actionName;
    std::string actionType;
    std::string goalResponseCallback;
    std::string feedbackCallback;
    std::string resultCallback;

    createActionClient_in();
};

struct createActionClient_out
{
    std::string actionClientHandle;

    createActionClient_out();
};

void createActionClient(SScriptCallBack *p, createActionClient_in *in, createActionClient_out *out);
std::string createActionClient(SScriptCallBack *p, std::string actionName, std::string actionType, std::string goalResponseCallback, std::string feedbackCallback, std::string resultCallback);
void createActionClient(SScriptCallBack *p, createActionClient_out *out, std::string actionName, std::string actionType, std::string goalResponseCallback, std::string feedbackCallback, std::string resultCallback);
void createActionClient_callback(SScriptCallBack *p);

struct shutdownActionClient_in
{
    SScriptCallBack _;
    std::string actionClientHandle;

    shutdownActionClient_in();
};

struct shutdownActionClient_out
{

    shutdownActionClient_out();
};

void shutdownActionClient(SScriptCallBack *p, shutdownActionClient_in *in, shutdownActionClient_out *out);
void shutdownActionClient(SScriptCallBack *p, std::string actionClientHandle);
void shutdownActionClient(SScriptCallBack *p, shutdownActionClient_out *out, std::string actionClientHandle);
void shutdownActionClient_callback(SScriptCallBack *p);

struct actionClientTreatUInt8ArrayAsString_in
{
    SScriptCallBack _;
    std::string actionClientHandle;

    actionClientTreatUInt8ArrayAsString_in();
};

struct actionClientTreatUInt8ArrayAsString_out
{

    actionClientTreatUInt8ArrayAsString_out();
};

void actionClientTreatUInt8ArrayAsString(SScriptCallBack *p, actionClientTreatUInt8ArrayAsString_in *in, actionClientTreatUInt8ArrayAsString_out *out);
void actionClientTreatUInt8ArrayAsString(SScriptCallBack *p, std::string actionClientHandle);
void actionClientTreatUInt8ArrayAsString(SScriptCallBack *p, actionClientTreatUInt8ArrayAsString_out *out, std::string actionClientHandle);
void actionClientTreatUInt8ArrayAsString_callback(SScriptCallBack *p);

struct sendGoal_in
{
    SScriptCallBack _;
    std::string actionClientHandle;

    sendGoal_in();
};

struct sendGoal_out
{
    bool success;

    sendGoal_out();
};

void sendGoal(SScriptCallBack *p, sendGoal_in *in, sendGoal_out *out);
bool sendGoal(SScriptCallBack *p, std::string actionClientHandle);
void sendGoal(SScriptCallBack *p, sendGoal_out *out, std::string actionClientHandle);
void sendGoal_callback(SScriptCallBack *p);

struct cancelLastGoal_in
{
    SScriptCallBack _;
    std::string actionClientHandle;

    cancelLastGoal_in();
};

struct cancelLastGoal_out
{
    bool success;

    cancelLastGoal_out();
};

void cancelLastGoal(SScriptCallBack *p, cancelLastGoal_in *in, cancelLastGoal_out *out);
bool cancelLastGoal(SScriptCallBack *p, std::string actionClientHandle);
void cancelLastGoal(SScriptCallBack *p, cancelLastGoal_out *out, std::string actionClientHandle);
void cancelLastGoal_callback(SScriptCallBack *p);

struct createActionServer_in
{
    SScriptCallBack _;
    std::string actionName;
    std::string actionType;
    std::string handleGoalCallback;
    std::string handleCancelCallback;
    std::string handleAcceptedCallback;

    createActionServer_in();
};

struct createActionServer_out
{
    std::string actionServerHandle;

    createActionServer_out();
};

void createActionServer(SScriptCallBack *p, createActionServer_in *in, createActionServer_out *out);
std::string createActionServer(SScriptCallBack *p, std::string actionName, std::string actionType, std::string handleGoalCallback, std::string handleCancelCallback, std::string handleAcceptedCallback);
void createActionServer(SScriptCallBack *p, createActionServer_out *out, std::string actionName, std::string actionType, std::string handleGoalCallback, std::string handleCancelCallback, std::string handleAcceptedCallback);
void createActionServer_callback(SScriptCallBack *p);

struct shutdownActionServer_in
{
    SScriptCallBack _;
    std::string actionServerHandle;

    shutdownActionServer_in();
};

struct shutdownActionServer_out
{

    shutdownActionServer_out();
};

void shutdownActionServer(SScriptCallBack *p, shutdownActionServer_in *in, shutdownActionServer_out *out);
void shutdownActionServer(SScriptCallBack *p, std::string actionServerHandle);
void shutdownActionServer(SScriptCallBack *p, shutdownActionServer_out *out, std::string actionServerHandle);
void shutdownActionServer_callback(SScriptCallBack *p);

struct actionServerTreatUInt8ArrayAsString_in
{
    SScriptCallBack _;
    std::string actionServerHandle;

    actionServerTreatUInt8ArrayAsString_in();
};

struct actionServerTreatUInt8ArrayAsString_out
{

    actionServerTreatUInt8ArrayAsString_out();
};

void actionServerTreatUInt8ArrayAsString(SScriptCallBack *p, actionServerTreatUInt8ArrayAsString_in *in, actionServerTreatUInt8ArrayAsString_out *out);
void actionServerTreatUInt8ArrayAsString(SScriptCallBack *p, std::string actionServerHandle);
void actionServerTreatUInt8ArrayAsString(SScriptCallBack *p, actionServerTreatUInt8ArrayAsString_out *out, std::string actionServerHandle);
void actionServerTreatUInt8ArrayAsString_callback(SScriptCallBack *p);

struct actionServerPublishFeedback_in
{
    SScriptCallBack _;
    std::string actionServerHandle;
    std::string goalUUID;

    actionServerPublishFeedback_in();
};

struct actionServerPublishFeedback_out
{

    actionServerPublishFeedback_out();
};

void actionServerPublishFeedback(SScriptCallBack *p, actionServerPublishFeedback_in *in, actionServerPublishFeedback_out *out);
void actionServerPublishFeedback(SScriptCallBack *p, std::string actionServerHandle, std::string goalUUID);
void actionServerPublishFeedback(SScriptCallBack *p, actionServerPublishFeedback_out *out, std::string actionServerHandle, std::string goalUUID);
void actionServerPublishFeedback_callback(SScriptCallBack *p);

struct actionServerActionAbort_in
{
    SScriptCallBack _;
    std::string actionServerHandle;
    std::string goalUUID;

    actionServerActionAbort_in();
};

struct actionServerActionAbort_out
{

    actionServerActionAbort_out();
};

void actionServerActionAbort(SScriptCallBack *p, actionServerActionAbort_in *in, actionServerActionAbort_out *out);
void actionServerActionAbort(SScriptCallBack *p, std::string actionServerHandle, std::string goalUUID);
void actionServerActionAbort(SScriptCallBack *p, actionServerActionAbort_out *out, std::string actionServerHandle, std::string goalUUID);
void actionServerActionAbort_callback(SScriptCallBack *p);

struct actionServerActionSucceed_in
{
    SScriptCallBack _;
    std::string actionServerHandle;
    std::string goalUUID;

    actionServerActionSucceed_in();
};

struct actionServerActionSucceed_out
{

    actionServerActionSucceed_out();
};

void actionServerActionSucceed(SScriptCallBack *p, actionServerActionSucceed_in *in, actionServerActionSucceed_out *out);
void actionServerActionSucceed(SScriptCallBack *p, std::string actionServerHandle, std::string goalUUID);
void actionServerActionSucceed(SScriptCallBack *p, actionServerActionSucceed_out *out, std::string actionServerHandle, std::string goalUUID);
void actionServerActionSucceed_callback(SScriptCallBack *p);

struct actionServerActionCanceled_in
{
    SScriptCallBack _;
    std::string actionServerHandle;
    std::string goalUUID;

    actionServerActionCanceled_in();
};

struct actionServerActionCanceled_out
{

    actionServerActionCanceled_out();
};

void actionServerActionCanceled(SScriptCallBack *p, actionServerActionCanceled_in *in, actionServerActionCanceled_out *out);
void actionServerActionCanceled(SScriptCallBack *p, std::string actionServerHandle, std::string goalUUID);
void actionServerActionCanceled(SScriptCallBack *p, actionServerActionCanceled_out *out, std::string actionServerHandle, std::string goalUUID);
void actionServerActionCanceled_callback(SScriptCallBack *p);

struct actionServerActionExecute_in
{
    SScriptCallBack _;
    std::string actionServerHandle;
    std::string goalUUID;

    actionServerActionExecute_in();
};

struct actionServerActionExecute_out
{

    actionServerActionExecute_out();
};

void actionServerActionExecute(SScriptCallBack *p, actionServerActionExecute_in *in, actionServerActionExecute_out *out);
void actionServerActionExecute(SScriptCallBack *p, std::string actionServerHandle, std::string goalUUID);
void actionServerActionExecute(SScriptCallBack *p, actionServerActionExecute_out *out, std::string actionServerHandle, std::string goalUUID);
void actionServerActionExecute_callback(SScriptCallBack *p);

struct actionServerActionIsCanceling_in
{
    SScriptCallBack _;
    std::string actionServerHandle;
    std::string goalUUID;

    actionServerActionIsCanceling_in();
};

struct actionServerActionIsCanceling_out
{
    bool result;

    actionServerActionIsCanceling_out();
};

void actionServerActionIsCanceling(SScriptCallBack *p, actionServerActionIsCanceling_in *in, actionServerActionIsCanceling_out *out);
bool actionServerActionIsCanceling(SScriptCallBack *p, std::string actionServerHandle, std::string goalUUID);
void actionServerActionIsCanceling(SScriptCallBack *p, actionServerActionIsCanceling_out *out, std::string actionServerHandle, std::string goalUUID);
void actionServerActionIsCanceling_callback(SScriptCallBack *p);

struct actionServerActionIsActive_in
{
    SScriptCallBack _;
    std::string actionServerHandle;
    std::string goalUUID;

    actionServerActionIsActive_in();
};

struct actionServerActionIsActive_out
{
    bool result;

    actionServerActionIsActive_out();
};

void actionServerActionIsActive(SScriptCallBack *p, actionServerActionIsActive_in *in, actionServerActionIsActive_out *out);
bool actionServerActionIsActive(SScriptCallBack *p, std::string actionServerHandle, std::string goalUUID);
void actionServerActionIsActive(SScriptCallBack *p, actionServerActionIsActive_out *out, std::string actionServerHandle, std::string goalUUID);
void actionServerActionIsActive_callback(SScriptCallBack *p);

struct actionServerActionIsExecuting_in
{
    SScriptCallBack _;
    std::string actionServerHandle;
    std::string goalUUID;

    actionServerActionIsExecuting_in();
};

struct actionServerActionIsExecuting_out
{
    bool result;

    actionServerActionIsExecuting_out();
};

void actionServerActionIsExecuting(SScriptCallBack *p, actionServerActionIsExecuting_in *in, actionServerActionIsExecuting_out *out);
bool actionServerActionIsExecuting(SScriptCallBack *p, std::string actionServerHandle, std::string goalUUID);
void actionServerActionIsExecuting(SScriptCallBack *p, actionServerActionIsExecuting_out *out, std::string actionServerHandle, std::string goalUUID);
void actionServerActionIsExecuting_callback(SScriptCallBack *p);

struct sendTransform_in
{
    SScriptCallBack _;

    sendTransform_in();
};

struct sendTransform_out
{

    sendTransform_out();
};

void sendTransform(SScriptCallBack *p, sendTransform_in *in, sendTransform_out *out);
void sendTransform(SScriptCallBack *p);
void sendTransform(SScriptCallBack *p, sendTransform_out *out);
void sendTransform_callback(SScriptCallBack *p);

struct sendTransforms_in
{
    SScriptCallBack _;

    sendTransforms_in();
};

struct sendTransforms_out
{

    sendTransforms_out();
};

void sendTransforms(SScriptCallBack *p, sendTransforms_in *in, sendTransforms_out *out);
void sendTransforms(SScriptCallBack *p);
void sendTransforms(SScriptCallBack *p, sendTransforms_out *out);
void sendTransforms_callback(SScriptCallBack *p);

struct imageTransportCreateSubscription_in
{
    SScriptCallBack _;
    std::string topicName;
    std::string topicCallback;
    int queueSize;

    imageTransportCreateSubscription_in();
};

struct imageTransportCreateSubscription_out
{
    std::string subscriptionHandle;

    imageTransportCreateSubscription_out();
};

void imageTransportCreateSubscription(SScriptCallBack *p, imageTransportCreateSubscription_in *in, imageTransportCreateSubscription_out *out);
std::string imageTransportCreateSubscription(SScriptCallBack *p, std::string topicName, std::string topicCallback, int queueSize = 1);
void imageTransportCreateSubscription(SScriptCallBack *p, imageTransportCreateSubscription_out *out, std::string topicName, std::string topicCallback, int queueSize = 1);
void imageTransportCreateSubscription_callback(SScriptCallBack *p);

struct imageTransportShutdownSubscription_in
{
    SScriptCallBack _;
    std::string subscriptionHandle;

    imageTransportShutdownSubscription_in();
};

struct imageTransportShutdownSubscription_out
{

    imageTransportShutdownSubscription_out();
};

void imageTransportShutdownSubscription(SScriptCallBack *p, imageTransportShutdownSubscription_in *in, imageTransportShutdownSubscription_out *out);
void imageTransportShutdownSubscription(SScriptCallBack *p, std::string subscriptionHandle);
void imageTransportShutdownSubscription(SScriptCallBack *p, imageTransportShutdownSubscription_out *out, std::string subscriptionHandle);
void imageTransportShutdownSubscription_callback(SScriptCallBack *p);

struct imageTransportCreatePublisher_in
{
    SScriptCallBack _;
    std::string topicName;
    int queueSize;

    imageTransportCreatePublisher_in();
};

struct imageTransportCreatePublisher_out
{
    std::string publisherHandle;

    imageTransportCreatePublisher_out();
};

void imageTransportCreatePublisher(SScriptCallBack *p, imageTransportCreatePublisher_in *in, imageTransportCreatePublisher_out *out);
std::string imageTransportCreatePublisher(SScriptCallBack *p, std::string topicName, int queueSize = 1);
void imageTransportCreatePublisher(SScriptCallBack *p, imageTransportCreatePublisher_out *out, std::string topicName, int queueSize = 1);
void imageTransportCreatePublisher_callback(SScriptCallBack *p);

struct imageTransportShutdownPublisher_in
{
    SScriptCallBack _;
    std::string publisherHandle;

    imageTransportShutdownPublisher_in();
};

struct imageTransportShutdownPublisher_out
{

    imageTransportShutdownPublisher_out();
};

void imageTransportShutdownPublisher(SScriptCallBack *p, imageTransportShutdownPublisher_in *in, imageTransportShutdownPublisher_out *out);
void imageTransportShutdownPublisher(SScriptCallBack *p, std::string publisherHandle);
void imageTransportShutdownPublisher(SScriptCallBack *p, imageTransportShutdownPublisher_out *out, std::string publisherHandle);
void imageTransportShutdownPublisher_callback(SScriptCallBack *p);

struct imageTransportPublish_in
{
    SScriptCallBack _;
    std::string publisherHandle;
    std::string data;
    int width;
    int height;
    std::string frame_id;

    imageTransportPublish_in();
};

struct imageTransportPublish_out
{

    imageTransportPublish_out();
};

void imageTransportPublish(SScriptCallBack *p, imageTransportPublish_in *in, imageTransportPublish_out *out);
void imageTransportPublish(SScriptCallBack *p, std::string publisherHandle, std::string data, int width, int height, std::string frame_id);
void imageTransportPublish(SScriptCallBack *p, imageTransportPublish_out *out, std::string publisherHandle, std::string data, int width, int height, std::string frame_id);
void imageTransportPublish_callback(SScriptCallBack *p);

struct getTime_in
{
    SScriptCallBack _;
    int clock_type;

    getTime_in();
};

struct getTime_out
{
    sim_ros2_time time;

    getTime_out();
};

void getTime(SScriptCallBack *p, getTime_in *in, getTime_out *out);
sim_ros2_time getTime(SScriptCallBack *p, int clock_type = sim_ros2_clock_ros);
void getTime(SScriptCallBack *p, getTime_out *out, int clock_type = sim_ros2_clock_ros);
void getTime_callback(SScriptCallBack *p);

struct getParamString_in
{
    SScriptCallBack _;
    std::string name;
    std::string defaultValue;

    getParamString_in();
};

struct getParamString_out
{
    bool exists;
    std::string value;

    getParamString_out();
};

void getParamString(SScriptCallBack *p, getParamString_in *in, getParamString_out *out);
void getParamString(SScriptCallBack *p, getParamString_out *out, std::string name, std::string defaultValue = "");
void getParamString_callback(SScriptCallBack *p);

struct getParamInt_in
{
    SScriptCallBack _;
    std::string name;
    int defaultValue;

    getParamInt_in();
};

struct getParamInt_out
{
    bool exists;
    int value;

    getParamInt_out();
};

void getParamInt(SScriptCallBack *p, getParamInt_in *in, getParamInt_out *out);
void getParamInt(SScriptCallBack *p, getParamInt_out *out, std::string name, int defaultValue = 0);
void getParamInt_callback(SScriptCallBack *p);

struct getParamDouble_in
{
    SScriptCallBack _;
    std::string name;
    double defaultValue;

    getParamDouble_in();
};

struct getParamDouble_out
{
    bool exists;
    double value;

    getParamDouble_out();
};

void getParamDouble(SScriptCallBack *p, getParamDouble_in *in, getParamDouble_out *out);
void getParamDouble(SScriptCallBack *p, getParamDouble_out *out, std::string name, double defaultValue = 0.0);
void getParamDouble_callback(SScriptCallBack *p);

struct getParamBool_in
{
    SScriptCallBack _;
    std::string name;
    bool defaultValue;

    getParamBool_in();
};

struct getParamBool_out
{
    bool exists;
    bool value;

    getParamBool_out();
};

void getParamBool(SScriptCallBack *p, getParamBool_in *in, getParamBool_out *out);
void getParamBool(SScriptCallBack *p, getParamBool_out *out, std::string name, bool defaultValue = false);
void getParamBool_callback(SScriptCallBack *p);

struct setParamString_in
{
    SScriptCallBack _;
    std::string name;
    std::string value;

    setParamString_in();
};

struct setParamString_out
{

    setParamString_out();
};

void setParamString(SScriptCallBack *p, setParamString_in *in, setParamString_out *out);
void setParamString(SScriptCallBack *p, std::string name, std::string value);
void setParamString(SScriptCallBack *p, setParamString_out *out, std::string name, std::string value);
void setParamString_callback(SScriptCallBack *p);

struct setParamInt_in
{
    SScriptCallBack _;
    std::string name;
    int value;

    setParamInt_in();
};

struct setParamInt_out
{

    setParamInt_out();
};

void setParamInt(SScriptCallBack *p, setParamInt_in *in, setParamInt_out *out);
void setParamInt(SScriptCallBack *p, std::string name, int value);
void setParamInt(SScriptCallBack *p, setParamInt_out *out, std::string name, int value);
void setParamInt_callback(SScriptCallBack *p);

struct setParamDouble_in
{
    SScriptCallBack _;
    std::string name;
    double value;

    setParamDouble_in();
};

struct setParamDouble_out
{

    setParamDouble_out();
};

void setParamDouble(SScriptCallBack *p, setParamDouble_in *in, setParamDouble_out *out);
void setParamDouble(SScriptCallBack *p, std::string name, double value);
void setParamDouble(SScriptCallBack *p, setParamDouble_out *out, std::string name, double value);
void setParamDouble_callback(SScriptCallBack *p);

struct setParamBool_in
{
    SScriptCallBack _;
    std::string name;
    bool value;

    setParamBool_in();
};

struct setParamBool_out
{

    setParamBool_out();
};

void setParamBool(SScriptCallBack *p, setParamBool_in *in, setParamBool_out *out);
void setParamBool(SScriptCallBack *p, std::string name, bool value);
void setParamBool(SScriptCallBack *p, setParamBool_out *out, std::string name, bool value);
void setParamBool_callback(SScriptCallBack *p);

struct hasParam_in
{
    SScriptCallBack _;
    std::string name;

    hasParam_in();
};

struct hasParam_out
{
    bool exists;

    hasParam_out();
};

void hasParam(SScriptCallBack *p, hasParam_in *in, hasParam_out *out);
bool hasParam(SScriptCallBack *p, std::string name);
void hasParam(SScriptCallBack *p, hasParam_out *out, std::string name);
void hasParam_callback(SScriptCallBack *p);

struct deleteParam_in
{
    SScriptCallBack _;
    std::string name;

    deleteParam_in();
};

struct deleteParam_out
{

    deleteParam_out();
};

void deleteParam(SScriptCallBack *p, deleteParam_in *in, deleteParam_out *out);
void deleteParam(SScriptCallBack *p, std::string name);
void deleteParam(SScriptCallBack *p, deleteParam_out *out, std::string name);
void deleteParam_callback(SScriptCallBack *p);

struct createInterface_in
{
    SScriptCallBack _;
    std::string type;

    createInterface_in();
};

struct createInterface_out
{

    createInterface_out();
};

void createInterface(SScriptCallBack *p, createInterface_in *in, createInterface_out *out);
void createInterface(SScriptCallBack *p, std::string type);
void createInterface(SScriptCallBack *p, createInterface_out *out, std::string type);
void createInterface_callback(SScriptCallBack *p);

struct getInterfaceConstants_in
{
    SScriptCallBack _;
    std::string type;

    getInterfaceConstants_in();
};

struct getInterfaceConstants_out
{

    getInterfaceConstants_out();
};

void getInterfaceConstants(SScriptCallBack *p, getInterfaceConstants_in *in, getInterfaceConstants_out *out);
void getInterfaceConstants(SScriptCallBack *p, std::string type);
void getInterfaceConstants(SScriptCallBack *p, getInterfaceConstants_out *out, std::string type);
void getInterfaceConstants_callback(SScriptCallBack *p);

struct supportedInterfaces_in
{
    SScriptCallBack _;

    supportedInterfaces_in();
};

struct supportedInterfaces_out
{
    std::vector< std::string > result;

    supportedInterfaces_out();
};

void supportedInterfaces(SScriptCallBack *p, supportedInterfaces_in *in, supportedInterfaces_out *out);
std::vector< std::string > supportedInterfaces(SScriptCallBack *p);
void supportedInterfaces(SScriptCallBack *p, supportedInterfaces_out *out);
void supportedInterfaces_callback(SScriptCallBack *p);

struct subscriptionCallback_in
{

    subscriptionCallback_in();
};

struct subscriptionCallback_out
{

    subscriptionCallback_out();
};

void subscriptionCallback(simInt scriptId, const char *func);
bool subscriptionCallback(simInt scriptId, const char *func, subscriptionCallback_in *in_args, subscriptionCallback_out *out_args);

struct imageTransportCallback_in
{
    std::string data;
    int width;
    int height;

    imageTransportCallback_in();
};

struct imageTransportCallback_out
{

    imageTransportCallback_out();
};

void imageTransportCallback(simInt scriptId, const char *func, std::string data, int width, int height);
bool imageTransportCallback(simInt scriptId, const char *func, imageTransportCallback_in *in_args, imageTransportCallback_out *out_args);

struct actionGoalResponseCallback_in
{
    std::string goalID;
    bool accepted;

    actionGoalResponseCallback_in();
};

struct actionGoalResponseCallback_out
{

    actionGoalResponseCallback_out();
};

void actionGoalResponseCallback(simInt scriptId, const char *func, std::string goalID, bool accepted);
bool actionGoalResponseCallback(simInt scriptId, const char *func, actionGoalResponseCallback_in *in_args, actionGoalResponseCallback_out *out_args);

// following functions must be implemented in the plugin

void createSubscription(SScriptCallBack *p, const char *cmd, createSubscription_in *in, createSubscription_out *out);
void shutdownSubscription(SScriptCallBack *p, const char *cmd, shutdownSubscription_in *in, shutdownSubscription_out *out);
void subscriptionTreatUInt8ArrayAsString(SScriptCallBack *p, const char *cmd, subscriptionTreatUInt8ArrayAsString_in *in, subscriptionTreatUInt8ArrayAsString_out *out);
void createPublisher(SScriptCallBack *p, const char *cmd, createPublisher_in *in, createPublisher_out *out);
void shutdownPublisher(SScriptCallBack *p, const char *cmd, shutdownPublisher_in *in, shutdownPublisher_out *out);
void publisherTreatUInt8ArrayAsString(SScriptCallBack *p, const char *cmd, publisherTreatUInt8ArrayAsString_in *in, publisherTreatUInt8ArrayAsString_out *out);
void publish(SScriptCallBack *p, const char *cmd, publish_in *in, publish_out *out);
void createClient(SScriptCallBack *p, const char *cmd, createClient_in *in, createClient_out *out);
void shutdownClient(SScriptCallBack *p, const char *cmd, shutdownClient_in *in, shutdownClient_out *out);
void clientTreatUInt8ArrayAsString(SScriptCallBack *p, const char *cmd, clientTreatUInt8ArrayAsString_in *in, clientTreatUInt8ArrayAsString_out *out);
void waitForService(SScriptCallBack *p, const char *cmd, waitForService_in *in, waitForService_out *out);
void call(SScriptCallBack *p, const char *cmd, call_in *in, call_out *out);
void createService(SScriptCallBack *p, const char *cmd, createService_in *in, createService_out *out);
void shutdownService(SScriptCallBack *p, const char *cmd, shutdownService_in *in, shutdownService_out *out);
void serviceTreatUInt8ArrayAsString(SScriptCallBack *p, const char *cmd, serviceTreatUInt8ArrayAsString_in *in, serviceTreatUInt8ArrayAsString_out *out);
void createActionClient(SScriptCallBack *p, const char *cmd, createActionClient_in *in, createActionClient_out *out);
void shutdownActionClient(SScriptCallBack *p, const char *cmd, shutdownActionClient_in *in, shutdownActionClient_out *out);
void actionClientTreatUInt8ArrayAsString(SScriptCallBack *p, const char *cmd, actionClientTreatUInt8ArrayAsString_in *in, actionClientTreatUInt8ArrayAsString_out *out);
void sendGoal(SScriptCallBack *p, const char *cmd, sendGoal_in *in, sendGoal_out *out);
void cancelLastGoal(SScriptCallBack *p, const char *cmd, cancelLastGoal_in *in, cancelLastGoal_out *out);
void createActionServer(SScriptCallBack *p, const char *cmd, createActionServer_in *in, createActionServer_out *out);
void shutdownActionServer(SScriptCallBack *p, const char *cmd, shutdownActionServer_in *in, shutdownActionServer_out *out);
void actionServerTreatUInt8ArrayAsString(SScriptCallBack *p, const char *cmd, actionServerTreatUInt8ArrayAsString_in *in, actionServerTreatUInt8ArrayAsString_out *out);
void actionServerPublishFeedback(SScriptCallBack *p, const char *cmd, actionServerPublishFeedback_in *in, actionServerPublishFeedback_out *out);
void actionServerActionAbort(SScriptCallBack *p, const char *cmd, actionServerActionAbort_in *in, actionServerActionAbort_out *out);
void actionServerActionSucceed(SScriptCallBack *p, const char *cmd, actionServerActionSucceed_in *in, actionServerActionSucceed_out *out);
void actionServerActionCanceled(SScriptCallBack *p, const char *cmd, actionServerActionCanceled_in *in, actionServerActionCanceled_out *out);
void actionServerActionExecute(SScriptCallBack *p, const char *cmd, actionServerActionExecute_in *in, actionServerActionExecute_out *out);
void actionServerActionIsCanceling(SScriptCallBack *p, const char *cmd, actionServerActionIsCanceling_in *in, actionServerActionIsCanceling_out *out);
void actionServerActionIsActive(SScriptCallBack *p, const char *cmd, actionServerActionIsActive_in *in, actionServerActionIsActive_out *out);
void actionServerActionIsExecuting(SScriptCallBack *p, const char *cmd, actionServerActionIsExecuting_in *in, actionServerActionIsExecuting_out *out);
void sendTransform(SScriptCallBack *p, const char *cmd, sendTransform_in *in, sendTransform_out *out);
void sendTransforms(SScriptCallBack *p, const char *cmd, sendTransforms_in *in, sendTransforms_out *out);
void imageTransportCreateSubscription(SScriptCallBack *p, const char *cmd, imageTransportCreateSubscription_in *in, imageTransportCreateSubscription_out *out);
void imageTransportShutdownSubscription(SScriptCallBack *p, const char *cmd, imageTransportShutdownSubscription_in *in, imageTransportShutdownSubscription_out *out);
void imageTransportCreatePublisher(SScriptCallBack *p, const char *cmd, imageTransportCreatePublisher_in *in, imageTransportCreatePublisher_out *out);
void imageTransportShutdownPublisher(SScriptCallBack *p, const char *cmd, imageTransportShutdownPublisher_in *in, imageTransportShutdownPublisher_out *out);
void imageTransportPublish(SScriptCallBack *p, const char *cmd, imageTransportPublish_in *in, imageTransportPublish_out *out);
void getTime(SScriptCallBack *p, const char *cmd, getTime_in *in, getTime_out *out);
void getParamString(SScriptCallBack *p, const char *cmd, getParamString_in *in, getParamString_out *out);
void getParamInt(SScriptCallBack *p, const char *cmd, getParamInt_in *in, getParamInt_out *out);
void getParamDouble(SScriptCallBack *p, const char *cmd, getParamDouble_in *in, getParamDouble_out *out);
void getParamBool(SScriptCallBack *p, const char *cmd, getParamBool_in *in, getParamBool_out *out);
void setParamString(SScriptCallBack *p, const char *cmd, setParamString_in *in, setParamString_out *out);
void setParamInt(SScriptCallBack *p, const char *cmd, setParamInt_in *in, setParamInt_out *out);
void setParamDouble(SScriptCallBack *p, const char *cmd, setParamDouble_in *in, setParamDouble_out *out);
void setParamBool(SScriptCallBack *p, const char *cmd, setParamBool_in *in, setParamBool_out *out);
void hasParam(SScriptCallBack *p, const char *cmd, hasParam_in *in, hasParam_out *out);
void deleteParam(SScriptCallBack *p, const char *cmd, deleteParam_in *in, deleteParam_out *out);
void createInterface(SScriptCallBack *p, const char *cmd, createInterface_in *in, createInterface_out *out);
void getInterfaceConstants(SScriptCallBack *p, const char *cmd, getInterfaceConstants_in *in, getInterfaceConstants_out *out);
void supportedInterfaces(SScriptCallBack *p, const char *cmd, supportedInterfaces_in *in, supportedInterfaces_out *out);

#endif // STUBS_H__INCLUDED
