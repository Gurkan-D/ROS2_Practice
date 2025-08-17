#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory.h>
#include "robot_interfaces/msg/robot_interface_raport.hpp"
#include "robot_interfaces/srv/set_system_periph.hpp"
#include <future>

#include "custom_common.hpp"
using namespace std::placeholders;
using namespace std::chrono_literals;

class UserInterface: public rclcpp::Node {
    std::string NodeName, PublisherName;
    rclcpp::Subscription<robot_interfaces::msg::RobotInterfaceRaport>::SharedPtr SystemSubscrber_;
    rclcpp::Client<robot_interfaces::srv::SetSystemPeriph>::SharedPtr client_handle_;
    void client_rsp_callback(rclcpp::Client<robot_interfaces::srv::SetSystemPeriph>::SharedFuture future);
    void system_update(robot_interfaces::msg::RobotInterfaceRaport msg);
    void client_request(int type, int val);
public:
    UserInterface(std::string nodename_,std::string publishername_): rclcpp::Node(nodename_), NodeName(nodename_),PublisherName(publishername_)
    {
        SystemSubscrber_ = this->create_subscription<robot_interfaces::msg::RobotInterfaceRaport>(PublisherName,10,std::bind(&UserInterface::system_update,this,_1));
        client_handle_ = this->create_client<robot_interfaces::srv::SetSystemPeriph>(std::string(USER_SERVICE_NAME));

    } 
};
void UserInterface::system_update(robot_interfaces::msg::RobotInterfaceRaport msg){
    int temperature = msg.temperature;
    std::string status = msg.debug_message;
    bool motor_stats = msg.motors_ready;
    if(temperature>90){
        RCLCPP_ERROR(this->get_logger(),"Overheat Temperature[%d]\nMotor is %s \n Additional: %s\n",temperature,motor_stats==true? "active":"inactive",status.c_str());
        client_request(Temperature,30);
    }else if (temperature>50){
        RCLCPP_WARN(this->get_logger(),"High Temperature[%d]\nMotor is %s \n Additional: %s\n",temperature,motor_stats==true? "active":"inactive",status.c_str());
    }else{
         RCLCPP_INFO(this->get_logger(),"Normal Temperature[%d]\nMotor is %s \n Additional: %s\n",temperature,motor_stats==true? "active":"inactive",status.c_str());
    }
}

void UserInterface::client_request(int type, int val){
    if(!client_handle_->wait_for_service(1s)){
        RCLCPP_ERROR(this->get_logger(),"Service is not ready!");
    }
    auto request = std::make_shared<robot_interfaces::srv::SetSystemPeriph::Request>();
    
    request->system_periph_name = type;
    request->val = val;
    client_handle_->async_send_request(request,std::bind(&UserInterface::client_rsp_callback,this,_1));
}   
void UserInterface::client_rsp_callback(rclcpp::Client<robot_interfaces::srv::SetSystemPeriph>::SharedFuture future){
    auto rsp = future.get();
    if(rsp->ret == 0) RCLCPP_INFO(this->get_logger(),"Succesfully updated requested value");
    else RCLCPP_ERROR(this->get_logger(),"Failed to set data...");
}
int main(int argc,char* argv[]){
    rclcpp::init(argc,argv);
    auto user = std::make_shared<UserInterface>("User","SystemStats");
    rclcpp::spin(user);
    rclcpp::shutdown();
}