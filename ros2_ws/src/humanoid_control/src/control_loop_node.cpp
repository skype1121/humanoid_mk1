#include <chrono>
#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int64.hpp"

class ControlLoopNode : public rclcpp::Node
{
public:
  ControlLoopNode()
  : Node("control_loop_node"), loop_count_(0U), control_period_ms_(1), log_interval_ticks_(1000),
    latest_command_(0.0)
  {
    // 제어 주기(ms) 파라미터를 선언하고 값을 읽는다.
    control_period_ms_ = this->declare_parameter<std::int64_t>("control_period_ms", 1);
    if (control_period_ms_ <= 0) {
      control_period_ms_ = 1;
    }

    // 로그 출력 주기(tick) 파라미터를 선언하고 값을 읽는다.
    log_interval_ticks_ = this->declare_parameter<std::int64_t>("log_interval_ticks", 1000);
    if (log_interval_ticks_ <= 0) {
      log_interval_ticks_ = 1000;
    }

    // 제어 루프 카운트를 퍼블리시할 토픽을 생성한다.
    publisher_ = this->create_publisher<std_msgs::msg::UInt64>("/control_loop_count", 10);

    // 최신 제어 명령 상태를 퍼블리시할 토픽을 생성한다.
    state_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/control_state", 10);

    // 외부 제어 명령을 수신할 토픽을 구독한다.
    command_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "/control_command",
      10,
      std::bind(&ControlLoopNode::on_command, this, std::placeholders::_1));

    // 파라미터로 설정한 주기(ms)의 제어 루프 타이머를 생성한다.
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(control_period_ms_),
      std::bind(&ControlLoopNode::on_timer, this));
  }

private:
  void on_timer()
  {
    // 루프 카운트를 증가시키고 매 콜백마다 퍼블리시한다.
    ++loop_count_;
    std_msgs::msg::UInt64 msg;
    msg.data = loop_count_;
    publisher_->publish(msg);

    // 최신 제어 명령 값을 상태 토픽으로 퍼블리시한다.
    std_msgs::msg::Float64 state_msg;
    state_msg.data = latest_command_;
    state_publisher_->publish(state_msg);

    // 파라미터로 설정한 tick 주기마다 로그를 출력한다.
    if (loop_count_ % static_cast<std::uint64_t>(log_interval_ticks_) == 0U) {
      RCLCPP_INFO(
        this->get_logger(),
        "control loop tick: %lu, latest_command: %.6f",
        loop_count_,
        latest_command_);
    }
  }

  void on_command(const std_msgs::msg::Float64::SharedPtr msg)
  {
    // 최신 제어 명령 값을 저장한다.
    latest_command_ = msg->data;
  }

  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr state_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr command_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::uint64_t loop_count_;
  std::int64_t control_period_ms_;
  std::int64_t log_interval_ticks_;
  double latest_command_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlLoopNode>());
  rclcpp::shutdown();
  return 0;
}
