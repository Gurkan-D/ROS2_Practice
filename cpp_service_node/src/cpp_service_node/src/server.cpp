#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <memory>  
#include <string>

using namespace std::placeholders;

class SimpleServer : public rclcpp::Node {
private:
std::string nodename_,servername_;

};

int main(){
    return 0;
}