#include "custom_msg_pkg/subscriber.hpp"

using std::placeholders::_1;
MytopicSubscriber::MytopicSubscriber() : Node("mytopic_subscriber")
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    mytopic_subscriber_ = this->create_subscription<my_topic::msg::MyTopic>(
            "topic",
            qos_profile,
            std::bind(&MytopicSubscriber::subscribe_mytopic_message, this, _1));
}

void MytopicSubscriber::subscribe_mytopic_message(const my_topic::msg::MyTopic::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "%ld is %s", msg->value, msg->name.c_str());
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MytopicSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}