#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "my_topic/msg/my_topic.hpp"

using namespace std::chrono_literals;

class MytopicPublisher : public rclcpp::Node
{
public:
    MytopicPublisher();
private:
    void publish_mytopic_msg();
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<my_topic::msg::MyTopic>::SharedPtr mytopic_publisher_;
    size_t count_;
};