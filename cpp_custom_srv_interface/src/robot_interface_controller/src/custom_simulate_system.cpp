#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory.h>
#include <chrono>
#include <std_msgs/msg/int32.hpp>
#include <random>
#include "custom_common.hpp"
#include "robot_interfaces/srv/set_system_periph.hpp"
using namespace std::chrono_literals;
using namespace std::placeholders;

class SubSystem : public rclcpp::Node {
private:
    std::string NodeName_, ReportName;
    int temperature_;
    bool direct_control;
    rclcpp::TimerBase::SharedPtr publish_interval_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr TemparaturePublisher;
    rclcpp::Service<robot_interfaces::srv::SetSystemPeriph>::SharedPtr   service_handle_;
    void service_callback(const robot_interfaces::srv::SetSystemPeriph::Request::SharedPtr request, 
                            robot_interfaces::srv::SetSystemPeriph::Response::SharedPtr rsp);   
    
    std::random_device rd;   //random generation
    std::mt19937 gen;
    std::uniform_int_distribution<>dist;

    void publish_temperature_callbck();
public:
    SubSystem(std::string nodename_,std::string publishername,int temp): rclcpp::Node(nodename_),NodeName_(nodename_),ReportName(publishername),temperature_(temp),
                                                                        gen(std::random_device{}()),dist(0,200)
    {
        direct_control=false;
        TemparaturePublisher = this->create_publisher<std_msgs::msg::Int32>(ReportName,10);
        publish_interval_ = this->create_wall_timer(0.5s,std::bind(&SubSystem::publish_temperature_callbck,this));
        service_handle_ = this->create_service<robot_interfaces::srv::SetSystemPeriph>(std::string(USER_SERVICE_NAME),std::bind(&SubSystem::service_callback,this,_1,_2));
        std::mt19937 gen(rd()); 
        std::uniform_int_distribution<>dist(0,200);
        
        // motor can be initialized here check stats and set errors etc..
    }
};

void SubSystem::service_callback(const robot_interfaces::srv::SetSystemPeriph::Request::SharedPtr request, 
                            robot_interfaces::srv::SetSystemPeriph::Response::SharedPtr rsp){

    auto periph_type = request->system_periph_name;
    auto periph_val = request->val;
    bool error =0;
    switch (periph_type)
    {
    case Motor:
        RCLCPP_INFO(this->get_logger(),"Motor set! %d",periph_val);
        break;
    case Temperature:
        RCLCPP_INFO(this->get_logger(),"Temperature set! %d",periph_val);
        temperature_ = periph_val;
        direct_control=true;
        break;
    case System:
            RCLCPP_WARN(this->get_logger(),"Requested Sytem Upgrade... %d",periph_val);// for the future it may hande restarting specific peripheral
            break;
    default:
            RCLCPP_ERROR(this->get_logger(),"Invalid request type.....");
            error=1;
        break;
    }

    rsp->ret=error;
}

void SubSystem::publish_temperature_callbck(){
    //for now lets generate a random number
    std_msgs::msg::Int32 tmp;
    if(direct_control){
        RCLCPP_WARN(this->get_logger(),"DirectControlActive[%d]",temperature_);
        tmp.data=temperature_;
        TemparaturePublisher->publish(tmp);
        return;
    }
    temperature_ =  dist(gen);
    tmp.data=temperature_;
    TemparaturePublisher->publish(tmp);

}
int main(int argc,char* argv[]){
    rclcpp::init(argc,argv);
    auto system = std::make_shared<SubSystem>("SubSystem","Subsystem_1",0);
    rclcpp::spin(system);
    rclcpp::shutdown();

}