#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <strings.h>
#include <iostream>
#include <chrono>

using namespace std::chrono_literals;
using namespace std::placeholders;


class SimpleSubscriber: public rclcpp::Node{
private:
    std::string nodename_,topicname_;
    int counter_;
    void subcriber_event_callback(const std_msgs::msg::String &msg);
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriptionhandle_;

public:
    SimpleSubscriber(std::string nodename,std::string topicname): 
    Node(nodename),nodename_(nodename),topicname_(topicname),counter_(0){
        RCLCPP_INFO(this->get_logger(),"Simple Subscriber[ %s ] initializing",nodename_.c_str());
        subscriptionhandle_ = this->create_subscription<std_msgs::msg::String>(this->topicname_,10,std::bind(&SimpleSubscriber::subcriber_event_callback,this,_1));
        RCLCPP_INFO(this->get_logger(),"Simple Subscriber[ %s ] initialized",nodename_.c_str());
    }
};

void SimpleSubscriber::subcriber_event_callback(const std_msgs::msg::String &msg){
    std::string tmp_msg = msg.data;
    counter_++;
    std::string out = "[ " + std::to_string(counter_)+"] "+tmp_msg;
        RCLCPP_INFO(this->get_logger(),"%s",out.c_str());
    
}

int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    auto subs = std::make_shared<SimpleSubscriber>("simpleSubscriber","Subtest");
    rclcpp::spin(subs);
    rclcpp::shutdown();

    return 0;
}