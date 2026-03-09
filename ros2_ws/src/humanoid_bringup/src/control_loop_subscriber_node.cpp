#include <cinttypes>
#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"

class ControlLoopSubscriberNode : public rclcpp::Node
{
public:
  ControlLoopSubscriberNode()
  : Node("control_loop_subscriber_node")
  {
    // 제어 루프 카운트 토픽을 구독한다.
    subscription_ = this->create_subscription<std_msgs::msg::UInt64>(
      "control_loop_count",
      10,
      std::bind(&ControlLoopSubscriberNode::on_message, this, std::placeholders::_1));
  }

private:
  void on_message(const std_msgs::msg::UInt64::SharedPtr msg) const
  {
    // 로그 과다를 방지하기 위해 1000 단위 값만 출력한다.
    if (msg->data % 1000U == 0U) {
      RCLCPP_INFO(this->get_logger(), "received control loop count: %" PRIu64, msg->data);
    }
  }

  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlLoopSubscriberNode>());
  rclcpp::shutdown();
  return 0;
}
