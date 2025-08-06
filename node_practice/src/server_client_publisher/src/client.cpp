#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>
#include <memory>  
#include <string>
#include <chrono>
#include <mutex>
#include <future>
using namespace std::placeholders;
using namespace std::chrono_literals;

//task is simple with publisher establish a service as well user will set requested values and that values will increase one by one
class SimpleClient : public rclcpp::Node {
private:
    std::string nodename_,publisher_name_,service_name_;
    int64_t number_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr Subscription_;
    rclcpp::Client<std_msgs::msg::Int64>::SharedPtr Client_handle;
    
    void client_callback(std_msgs::msg::Int64 msg);
public:
    SimpleClient(std::string nodename,std::string publisher_name,std::string service_name,int64_t initval):
        Node(nodename),nodename_{nodename},publisher_name_{publisher_name},service_name_{service_name}
        {
            Subscription_ = this->create_subscription<std_msgs::msg::Int64>(publisher_name_,10,std::bind(&SimpleClient::client_callback,this,_1,_2));
            Client_handle = this->create_client<std_msgs::msg::Int64>(service_name_,10);
            RCLCPP_INFO(this->get_logger(),"Subscriber started!");
            while(!Client_handle->wait_for_service(1s)&&rclcpp::ok()){
            RCLCPP_WARN(this->get_logger(),"Waiting for server...");
            }
            // auto req = std::make_shared <std_msgs::msg::Int64>();
            // req->data=initval;
            // auto rsp =Client_handle->async_send_request(req);
            
        }

};

void SimpleClient::client_callback(std_msgs::msg::Int64 msg){

    RCLCPP_INFO(this->get_logger(), "Got response %ld",msg);
}

int main(int argc,char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SimpleClient>("NumbersClientSub","NumberPublisher","SetNumber");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}