#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <string>
#include <memory>


using namespace std::placeholders;

class SimpleServer : public rclcpp::Node{


private :
std::string nodename_,servername_;
rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr srv_handle_;
void service_callback(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                                    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response);
public:
    SimpleServer(std::string Nodename,std::string Servername): Node(Nodename),nodename_(Nodename),servername_(Servername){
        srv_handle_ = this->create_service<example_interfaces::srv::AddTwoInts>(servername_,
                                                                                std::bind(&SimpleServer::service_callback,this,_1,_2));
    RCLCPP_INFO(this->get_logger(), "Service Has started");
    }

};

void SimpleServer::service_callback(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                                    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response){


    response->sum = request->a+request->b;
    RCLCPP_INFO(this->get_logger(),"Incoming request\na: %ld" " b: %ld",request->a,request->b);
    RCLCPP_INFO(this->get_logger(), "sending back response: [%ld]", (long int)response->sum);

}

int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    auto obj = std::make_shared<SimpleServer>("add_two_ints","add_two_ints");
    rclcpp::spin(obj);
    rclcpp::shutdown();
}


