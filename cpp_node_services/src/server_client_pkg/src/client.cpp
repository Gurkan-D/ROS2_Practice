#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <string>
#include <memory>
#include <chrono>   
#include <future>

using namespace std::placeholders;
using namespace std::chrono_literals;
class SimpleClient : public rclcpp::Node{
private :
std::string nodename_,clientname_;
rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr clienthandle_;
void client_callback(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future);

public:
    SimpleClient(std::string Nodename,std::string Clientname): Node(Nodename),nodename_(Nodename),clientname_(Clientname){
        clienthandle_ = this->create_client<example_interfaces::srv::AddTwoInts>(clientname_);//,10,
                                                                                // std::bind(&SimpleClient::client_callback,this,_1,_2));
    RCLCPP_INFO(this->get_logger(), "Client Has started");
    }
    void client_request(int a,int b);
};

void SimpleClient::client_request(int a,int b){
    while(!clienthandle_->wait_for_service(1s)&&rclcpp::ok()){
        RCLCPP_WARN(this->get_logger(),"Waiting for server...");
    }
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a=a;
    request->b=b;
    clienthandle_->async_send_request(request,
        std::bind(&SimpleClient::client_callback,this,_1));
}
void SimpleClient::client_callback(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future){
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Got response %d",(int)response->sum);
}

int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    auto obj = std::make_shared<SimpleClient>("add_two_ints_client","add_two_ints");
    obj->client_request(10,15);
    rclcpp::spin(obj);
    rclcpp::shutdown();
}


