#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "my_topic/msg/my_topic.hpp"

using std::placeholders::_1;

class MytopicSubscriber : public rclcpp::Node
{
public:
    MytopicSubscriber();

private:
    void subscribe_mytopic_message(const my_topic::msg::MyTopic::SharedPtr msg) const;
    rclcpp::Subscription<my_topic::msg::MyTopic>::SharedPtr mytopic_subscriber_;
};