#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <strings.h>
#include <iostream>
#include <chrono>

using namespace std::chrono_literals;

class SimplePublisher: public rclcpp::Node{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr SimplePublisher_;
    std::string name_;
    int counter_;
    std::string topic_name_;
    void timer_callback();
public:
    SimplePublisher(std::string nodename,std::string topicname):rclcpp::Node{nodename},name_(nodename),counter_{0},topic_name_{topicname}{
        RCLCPP_INFO(this->get_logger()," Publisher is starting");
        SimplePublisher_ = this->create_publisher<std_msgs::msg::String>(this->topic_name_,10);
        timer_ = this->create_wall_timer(1s,std::bind(&SimplePublisher::timer_callback,this));
        RCLCPP_INFO(this->get_logger()," Publisher is started");
    }

};

void SimplePublisher::timer_callback(){
    counter_++;
    auto msg = std_msgs::msg::String();
    msg.data = std::to_string(counter_) + " times published!";
    this->SimplePublisher_->publish(msg);
}


int main(int argc, char** argv){ 
    rclcpp::init(argc,argv);
    auto publishernode = std::make_shared<SimplePublisher>("simplePublisher","pubTest");
    rclcpp::spin(publishernode);
    rclcpp::shutdown();
    return 0;
}