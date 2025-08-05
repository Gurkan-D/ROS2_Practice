#include <rclcpp/rclcpp.hpp>
#include <strings.h>
#include <iostream>
#include <chrono>

using namespace std::chrono_literals;

class TestNode : public rclcpp::Node {

private:

    std::string _nodeName;
    void timer_callback();
    rclcpp::TimerBase::SharedPtr _timer;
public:
    TestNode(std::string nodename):Node(nodename),_nodeName{nodename}{
        RCLCPP_INFO(this->get_logger()," Simple Node started");
        
        _timer = this->create_wall_timer(1s,std::bind(&TestNode::timer_callback,this));
    }



};

void TestNode::timer_callback(){
    RCLCPP_INFO(this->get_logger()," %s working",this->_nodeName.c_str());
}
int main(int argc , char** argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TestNode>("SimpleNode");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}