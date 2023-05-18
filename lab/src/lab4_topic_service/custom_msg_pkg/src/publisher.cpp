#include "custom_msg_pkg/publisher.hpp"

MytopicPublisher::MytopicPublisher() : Node("mytopic_publisher"), count_(0)
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    mytopic_publisher_ = this->create_publisher<my_topic::msg::MyTopic>(
            "topic", qos_profile);
    timer_ = this->create_wall_timer(
            1s, std::bind(&MytopicPublisher::publish_mytopic_msg, this));
}
void MytopicPublisher::publish_mytopic_msg()
{
    auto msg = my_topic::msg::MyTopic();

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist(1, 9); 
    msg.value = dist(gen);

    if (msg.value % 2 == 0) {
        msg.name = "even";
    } else {
        msg.name = "odd";
    }
    RCLCPP_INFO(this->get_logger(), "Published message: (%ld)", msg.value);
    mytopic_publisher_->publish(msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MytopicPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}