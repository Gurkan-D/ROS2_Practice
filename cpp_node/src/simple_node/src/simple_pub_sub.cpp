#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <deque> // normal queue is also okay, I just want to use it
#include <iostream>
#include <chrono>

#include <mutex>

using namespace std::placeholders;
using namespace std::chrono_literals;

class SimplePubSub:public rclcpp::Node{
private:
    std::string nodename_,pubtopic_,subtopic_;
    bool flag_timer_en_;
    std::mutex mutex_; // can be added to watchdog timer to track mutex is relasing or not but for 2 node and topic I will not do it
    void pubsubcallback(std_msgs::msg::String msg); //receive callback
    void publishcallback();
    std::deque<std_msgs::msg::String> deq_buffer_;//double ended queue

    rclcpp::TimerBase::SharedPtr timer_; // since it will be callback to publish so it is not necessary for now
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr     pub_;
public:
    SimplePubSub(std::string nodename,std::string pubtopic,std::string subtopic,bool flag_timer_en):
        rclcpp::Node(nodename),nodename_(nodename),pubtopic_(pubtopic),subtopic_(subtopic),flag_timer_en_(flag_timer_en)
    {
        sub_    =   this->create_subscription<std_msgs::msg::String>(this->subtopic_,10,std::bind(&SimplePubSub::pubsubcallback,this,_1));
        pub_    =   this->create_publisher< std_msgs::msg::String>(this->pubtopic_,10);
        if(flag_timer_en){
            timer_ = this->create_wall_timer(10s,std::bind(&SimplePubSub::publishcallback,this));
        }
    }
};
void SimplePubSub::pubsubcallback(std_msgs::msg::String msg) //receive callback
{
    if(this->flag_timer_en_){
        this->mutex_.lock();
        this->deq_buffer_.push_back(msg);
        this->mutex_.unlock();
        return; // return without jumping publishing stage
    }
    // driven by interrupt
    RCLCPP_INFO(this->get_logger(),"Data received %s transmitting...",msg.data.c_str());
    this->pub_->publish(msg);

}
void SimplePubSub::publishcallback(){
    this->mutex_.lock();
    std::string tmp = "";
    int i=1;
    while(this->deq_buffer_.size()){
        tmp += std::to_string(i)+". "+this->deq_buffer_.front().data+"\n";
        this->deq_buffer_.pop_front();
        i++;
    }
    // deq_buffer_.clear();
    this->mutex_.unlock(); // job done

    std_msgs::msg::String rsp;
    rsp.data = tmp;
    this->pub_->publish(rsp);
    
}

int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    auto obj = std::make_shared<SimplePubSub>("Simple_PubSub","Subtest","pubTest",true);
    rclcpp::spin(obj);
    rclcpp::shutdown();
    return 0;
}