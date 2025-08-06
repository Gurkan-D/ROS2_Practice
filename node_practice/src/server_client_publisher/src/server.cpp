#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>
#include <memory>  
#include <string>
#include <chrono>
#include <mutex>
using namespace std::placeholders;
using namespace std::chrono_literals;

//task is simple with publisher establish a service as well user will set requested values and that values will increase one by one
class SimpleService : public rclcpp::Node {
private:
    std::string nodename_,publisher_name_,service_name_;
    int64_t number_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_handle_;
    rclcpp::Service<std_msgs::msg::Int64>::SharedPtr service_handle_;
    rclcpp::TimerBase::SharedPtr timer;
    pthread_mutexattr_t mutex_attr;
    pthread_mutex_t cute_mutex; // I could use std::mutex but I am chilling so I want to use pthread yeey
    void service_callback_(std_msgs::msg::Int64 msg);
    void publish_timer_callback_();
public:
    SimpleService(std::string nodename,std::string publisher_name,std::string service_name):
        Node(nodename),nodename_{nodename},publisher_name_{publisher_name},service_name_{service_name}
        {
            int rc = pthread_mutexattr_init(&mutex_attr);
            assert(rc != -1 && "ATTR ERROR USE std::mutex why you are tring this");
            rc = pthread_mutex_init(&cute_mutex,0);
            assert(rc != -1 && "pthread_mutex_init because it is fun, ");
            publisher_handle_ = this->create_publisher<std_msgs::msg::Int64>(publisher_name_,10);
            service_handle_ = this->create_service<std_msgs::msg::Int64>(service_name_,std::bind(&SimpleService::service_callback_,this,_1,_2),10);
            timer = this->create_wall_timer(1s,std::bind(&SimpleService::publish_timer_callback_,this));
        }

};

void SimpleService::service_callback_(std_msgs::msg::Int64 msg){
    pthread_mutex_trylock(&cute_mutex);
    RCLCPP_INFO(this->get_logger(),"Ora! data %ld",msg.data);
    //pure fantasy I decided to use mutex here to protect shared resource
    if (pthread_mutex_trylock(&cute_mutex) != 0)                                       
    if (errno == EBUSY)                                                         
      RCLCPP_WARN(this->get_logger(),"service doing something try again!");                            
    else {     
        assert(0 && " Something wrong with code");                                                                 
    }                                                                           
    else number_ = msg.data;
    pthread_mutex_unlock(&cute_mutex);
    RCLCPP_WARN(this->get_logger(),"Value set to %ld",msg.data);
}
void SimpleService::publish_timer_callback_(){
    if (pthread_mutex_trylock(&cute_mutex) != 0)                                       
    if (errno == EBUSY)                                                         
      RCLCPP_WARN(this->get_logger(),"service doing something try again!");                            
    else {     
        assert(0 && " Something wrong with code");                                                                 
    }                                                                           
    else number_++;
    pthread_mutex_unlock(&cute_mutex);
    std_msgs::msg::Int64 tmp;
    tmp.data=number_;
    publisher_handle_->publish(tmp);
}
int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SimpleService>("Numbers","NumberPublisher","SetNumber");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}