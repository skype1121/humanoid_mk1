#include <chrono>
#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"

class ControlLoopNode : public rclcpp::Node
{
public:
  ControlLoopNode()
  : Node("control_loop_node"), loop_count_(0U)
  {
    // 제어 루프 카운트를 퍼블리시할 토픽을 생성한다.
    publisher_ = this->create_publisher<std_msgs::msg::UInt64>("control_loop_count", 10);

    // 1ms 주기의 제어 루프 타이머를 생성한다.
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&ControlLoopNode::on_timer, this));
  }

private:
  void on_timer()
  {
    // 루프 카운트를 증가시키고 1000회마다 상태를 출력한다.
    ++loop_count_;

    // 현재 루프 카운트를 최소 메시지 형태로 퍼블리시한다.
    std_msgs::msg::UInt64 msg;
    msg.data = loop_count_;
    publisher_->publish(msg);

    if (loop_count_ % 1000U == 0U) {
      RCLCPP_INFO(this->get_logger(), "control loop tick: %lu", loop_count_);
    }
  }

  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::uint64_t loop_count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlLoopNode>());
  rclcpp::shutdown();
  return 0;
}
