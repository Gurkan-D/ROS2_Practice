#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/client.hpp>
#include <string>
#include <memory.h>
#include <chrono>
#include <std_msgs/msg/int32.hpp>
#include "robot_interfaces/msg/robot_interface_raport.hpp"
#include "robot_interfaces/srv/set_system_periph.hpp"
#include "custom_common.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;



class RobotInterfacePublisher: public rclcpp::Node{

private:
    std::string NodeName_,PublisherName_,SubsystemName_;
    robot_interfaces::msg::RobotInterfaceRaport rpt;
    rclcpp::Publisher<robot_interfaces::msg::RobotInterfaceRaport>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subsys_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::Service<robot_interfaces::srv::SetSystemPeriph>::SharedPtr   service_handle_;
    rclcpp::Client<robot_interfaces::srv::SetSystemPeriph>::SharedPtr client_handle_;

    bool flag_waitingfor_response=false;
    void service_callback(const robot_interfaces::srv::SetSystemPeriph::Request::SharedPtr request, 
                            robot_interfaces::srv::SetSystemPeriph::Response::SharedPtr rsp);   
    void publisher_callback();
    void subsys_callback(std_msgs::msg::Int32 msg);
    void client_rsp_callback(rclcpp::Client<robot_interfaces::srv::SetSystemPeriph>::SharedFuture future);
    void client_request(int type, int val);
public:
    RobotInterfacePublisher(std::string NodeName,std::string PublisherName,std::string Subsys): Node(NodeName),NodeName_(NodeName),PublisherName_(PublisherName),SubsystemName_(Subsys)
    {
        rpt.debug_message="System is Not initialized";
        rpt.motors_ready=false;
        rpt.temperature=0; // asume temprature paramters

        publisher_ = this->create_publisher<robot_interfaces::msg::RobotInterfaceRaport>(PublisherName_,10);
        publish_timer_ = this->create_wall_timer(1s,std::bind(&RobotInterfacePublisher::publisher_callback,this));
        service_handle_ = this->create_service<robot_interfaces::srv::SetSystemPeriph>(std::string(USER_SERVICE_NAME),std::bind(&RobotInterfacePublisher::service_callback,this,_1,_2));
        subsys_ = this->create_subscription<std_msgs::msg::Int32>(SubsystemName_,10,std::bind(&RobotInterfacePublisher::subsys_callback,this,_1));
        
        client_handle_ = this->create_client<robot_interfaces::srv::SetSystemPeriph>(std::string(SUBSYSTEM_SERVICE_NAME));

    }
};


void RobotInterfacePublisher::client_request(int type, int val){
    if(flag_waitingfor_response) return;
    if(!client_handle_->wait_for_service(1s)){
        RCLCPP_ERROR(this->get_logger(),"Service is not ready!");
        return;
    }
    auto request = std::make_shared<robot_interfaces::srv::SetSystemPeriph::Request>();
    
    request->system_periph_name = type;
    request->val = val;
    flag_waitingfor_response=true;
    client_handle_->async_send_request(request,std::bind(&RobotInterfacePublisher::client_rsp_callback,this,_1));
}   
void RobotInterfacePublisher::client_rsp_callback(rclcpp::Client<robot_interfaces::srv::SetSystemPeriph>::SharedFuture future){
    auto rsp = future.get();
    if(rsp->ret == 0) RCLCPP_INFO(this->get_logger(),"Succesfully updated requested value");
    else RCLCPP_ERROR(this->get_logger(),"Failed to set data...");
    flag_waitingfor_response=false;
}

void RobotInterfacePublisher::service_callback(const robot_interfaces::srv::SetSystemPeriph::Request::SharedPtr request, 
                            robot_interfaces::srv::SetSystemPeriph::Response::SharedPtr rsp){

    auto periph_type = request->system_periph_name;
    auto periph_val = request->val;
    bool error=false;
    switch (periph_type)
    {
    case Motor:
        if(periph_val >=0 && periph_val<=100) RCLCPP_INFO(this->get_logger(),"Motor speed will be set to %d",periph_val);
        else 
            {
                RCLCPP_ERROR(this->get_logger(),"Error! Motor[%d] is not in the limit [0-100]",periph_val);
                error=true;
            }
        break;
    case Temperature:
            if(periph_val >=0 && periph_val<=200) RCLCPP_INFO(this->get_logger(),"Motor will wait till temperature reaches to %d",periph_val);
        else 
            {
                RCLCPP_ERROR(this->get_logger(),"Error! Temperature[%d] is not in the limit [0-200]",periph_val);
                error=true;
            }
        break;
    case System:
            if(periph_val == 1) RCLCPP_INFO(this->get_logger(),"system reset type...[%d]",periph_val); // for the future it may hande restarting specific peripheral
            break;
    default:
            RCLCPP_ERROR(this->get_logger(),"Invalid request type.....");
            error=true;
        break;
    }

    if(!error)client_request(periph_type,periph_val);

    rsp->ret=error;
}
void RobotInterfacePublisher::publisher_callback(){
    publisher_->publish(rpt);
}
void RobotInterfacePublisher::subsys_callback(std_msgs::msg::Int32 msg)
{
    
    rpt.temperature = msg.data;
    if(rpt.temperature>90){
        rpt.debug_message.clear();
        rpt.debug_message = "OverHeat Halting motor";
        rpt.motors_ready=false;
    }else if (rpt.temperature>50){
        rpt.debug_message.clear();
        rpt.debug_message = "Reduce Speed";
        rpt.motors_ready=true;
    }else{
        rpt.debug_message.clear();
        rpt.debug_message = "ok";
        rpt.motors_ready=true;
    }
}

int main(int argc,char* argv[]){
    rclcpp::init(argc,argv);
    auto obj = std::make_shared<RobotInterfacePublisher>("MainControllerNode","SystemStats","Subsystem_1"); 
    rclcpp::spin(obj);
    rclcpp::shutdown();
    return 0;
}