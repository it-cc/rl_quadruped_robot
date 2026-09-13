#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64_multi_array.hpp"

namespace
{
constexpr std::size_t kPingFieldCount = 2;
constexpr std::size_t kPongFieldCount = 5;

uint64_t systemTimeNs()
{
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
}

double percentile(std::vector<double> values, const double quantile)
{
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();

  std::sort(values.begin(), values.end());
  const double position = quantile * static_cast<double>(values.size() - 1U);
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  const double fraction = position - static_cast<double>(lower);
  return values[lower] + fraction * (values[upper] - values[lower]);
}

double mean(const std::vector<double>& values)
{
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  return std::accumulate(values.begin(), values.end(), 0.0) /
         static_cast<double>(values.size());
}
}  // namespace

class LatencyTestNode : public rclcpp::Node
{
 public:
  LatencyTestNode() : Node("microros_latency_test")
  {
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
    const int warmup_samples = declare_parameter<int>("warmup_samples", 100);
    const int sample_count = declare_parameter<int>("sample_count", 3000);
    response_timeout_ms_ = declare_parameter<int>("response_timeout_ms", 2000);
    max_valid_latency_ms_ =
        declare_parameter<double>("max_valid_latency_ms", 1000.0);
    csv_path_ =
        declare_parameter<std::string>("csv_path", "latency_results.csv");

    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0 ||
        warmup_samples < 0 || sample_count <= 0 || response_timeout_ms_ <= 0 ||
        !std::isfinite(max_valid_latency_ms_) || max_valid_latency_ms_ <= 0.0)
    {
      throw std::invalid_argument("invalid latency test parameter");
    }

    warmup_samples_ = static_cast<std::size_t>(warmup_samples);
    sample_count_ = static_cast<std::size_t>(sample_count);
    total_send_count_ = warmup_samples_ + sample_count_;
    max_valid_latency_ns_ =
        static_cast<uint64_t>(max_valid_latency_ms_ * 1000000.0);

    csv_.open(csv_path_, std::ios::out | std::ios::trunc);
    if (!csv_.is_open())
      throw std::runtime_error("cannot open CSV file: " + csv_path_);
    csv_ << "sequence,pc_send_ns,mcu_receive_ns,mcu_send_ns,pc_receive_ns,"
            "pc_to_mcu_ms,mcu_to_pc_ms,time_synchronized,valid\n";
    csv_ << std::fixed << std::setprecision(6);

    publisher_ = create_publisher<std_msgs::msg::UInt64MultiArray>(
        "latency_test/ping", rclcpp::QoS(10).reliable());
    subscription_ = create_subscription<std_msgs::msg::UInt64MultiArray>(
        "latency_test/pong", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::UInt64MultiArray::SharedPtr message)
        { receivePong(*message); });

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_));
    publish_timer_ = create_wall_timer(period, [this]() { publishPing(); });

    RCLCPP_INFO(get_logger(),
                "Starting: %.3f Hz, %zu warm-up + %zu measured samples, CSV=%s",
                publish_rate_hz_, warmup_samples_, sample_count_,
                csv_path_.c_str());
  }

 private:
  struct PendingPing
  {
    uint64_t send_time_ns;
    bool measured;
  };

  void publishPing()
  {
    if (sent_count_ >= total_send_count_) return;

    std_msgs::msg::UInt64MultiArray ping;
    ping.data.resize(kPingFieldCount);
    const uint64_t sequence = static_cast<uint64_t>(sent_count_);
    const uint64_t send_time_ns = systemTimeNs();
    ping.data[0] = sequence;
    ping.data[1] = send_time_ns;
    pending_.emplace(sequence,
                     PendingPing{send_time_ns, sent_count_ >= warmup_samples_});
    publisher_->publish(ping);
    ++sent_count_;

    if (sent_count_ == total_send_count_)
    {
      publish_timer_->cancel();
      finish_timer_ =
          create_wall_timer(std::chrono::milliseconds(response_timeout_ms_),
                            [this]()
                            {
                              finish_timer_->cancel();
                              finish();
                            });
      RCLCPP_INFO(get_logger(),
                  "All %zu pings sent; waiting %d ms for remaining pongs",
                  total_send_count_, response_timeout_ms_);
    }
  }

  void receivePong(const std_msgs::msg::UInt64MultiArray& pong)
  {
    const uint64_t pc_receive_ns = systemTimeNs();
    if (finished_) return;
    if (pong.data.size() < kPongFieldCount)
    {
      ++malformed_count_;
      return;
    }

    const uint64_t sequence = pong.data[0];
    if (!responded_sequences_.insert(sequence).second)
    {
      ++duplicate_count_;
      return;
    }

    const auto pending = pending_.find(sequence);
    if (pending == pending_.end())
    {
      ++unmatched_count_;
      writeCsvRow(pong, pc_receive_ns, 0.0, 0.0, false);
      return;
    }

    const PendingPing sent = pending->second;
    pending_.erase(pending);
    ++matched_count_;

    const uint64_t pc_send_ns = pong.data[1];
    const uint64_t mcu_receive_ns = pong.data[2];
    const uint64_t mcu_send_ns = pong.data[3];
    const bool synchronized = pong.data[4] != 0U;
    bool valid = synchronized && pc_send_ns == sent.send_time_ns &&
                 mcu_receive_ns >= pc_send_ns &&
                 mcu_send_ns >= mcu_receive_ns && pc_receive_ns >= mcu_send_ns;

    double pc_to_mcu_ms = 0.0;
    double mcu_to_pc_ms = 0.0;
    if (valid)
    {
      const uint64_t pc_to_mcu_ns = mcu_receive_ns - pc_send_ns;
      const uint64_t mcu_to_pc_ns = pc_receive_ns - mcu_send_ns;
      valid = pc_to_mcu_ns <= max_valid_latency_ns_ &&
              mcu_to_pc_ns <= max_valid_latency_ns_;
      pc_to_mcu_ms = static_cast<double>(pc_to_mcu_ns) / 1000000.0;
      mcu_to_pc_ms = static_cast<double>(mcu_to_pc_ns) / 1000000.0;
    }

    if (!synchronized) ++unsynchronized_count_;
    if (!valid) ++invalid_count_;
    if (valid && sent.measured)
    {
      pc_to_mcu_samples_ms_.push_back(pc_to_mcu_ms);
      mcu_to_pc_samples_ms_.push_back(mcu_to_pc_ms);
    }
    writeCsvRow(pong, pc_receive_ns, pc_to_mcu_ms, mcu_to_pc_ms, valid);
  }

  void writeCsvRow(const std_msgs::msg::UInt64MultiArray& pong,
                   const uint64_t pc_receive_ns, const double pc_to_mcu_ms,
                   const double mcu_to_pc_ms, const bool valid)
  {
    csv_ << pong.data[0] << ',' << pong.data[1] << ',' << pong.data[2] << ','
         << pong.data[3] << ',' << pc_receive_ns << ',' << pc_to_mcu_ms << ','
         << mcu_to_pc_ms << ',' << (pong.data[4] != 0U ? 1 : 0) << ','
         << (valid ? 1 : 0) << '\n';
  }

  void logDirection(const char* name, const std::vector<double>& values) const
  {
    RCLCPP_INFO(get_logger(),
                "%s [ms]: n=%zu mean=%.3f median=%.3f P95=%.3f P99=%.3f "
                "max=%.3f",
                name, values.size(), mean(values), percentile(values, 0.50),
                percentile(values, 0.95), percentile(values, 0.99),
                values.empty()
                    ? std::numeric_limits<double>::quiet_NaN()
                    : *std::max_element(values.begin(), values.end()));
  }

  void finish()
  {
    if (finished_) return;
    finished_ = true;
    csv_.flush();

    const std::size_t losses = total_send_count_ - matched_count_;
    RCLCPP_INFO(get_logger(), "Latency test complete; results: %s",
                csv_path_.c_str());
    logDirection("PC -> MCU", pc_to_mcu_samples_ms_);
    logDirection("MCU -> PC", mcu_to_pc_samples_ms_);
    RCLCPP_INFO(get_logger(),
                "sent=%zu matched=%zu losses=%zu duplicates=%zu malformed=%zu "
                "unmatched=%zu unsynchronized=%zu invalid=%zu",
                total_send_count_, matched_count_, losses, duplicate_count_,
                malformed_count_, unmatched_count_, unsynchronized_count_,
                invalid_count_);
    rclcpp::shutdown();
  }

  double publish_rate_hz_{50.0};
  double max_valid_latency_ms_{1000.0};
  int response_timeout_ms_{2000};
  std::size_t warmup_samples_{100};
  std::size_t sample_count_{3000};
  std::size_t total_send_count_{3100};
  std::size_t sent_count_{0};
  std::size_t matched_count_{0};
  std::size_t duplicate_count_{0};
  std::size_t malformed_count_{0};
  std::size_t unmatched_count_{0};
  std::size_t unsynchronized_count_{0};
  std::size_t invalid_count_{0};
  uint64_t max_valid_latency_ns_{1000000000ULL};
  bool finished_{false};
  std::string csv_path_;
  std::ofstream csv_;
  std::unordered_map<uint64_t, PendingPing> pending_;
  std::unordered_set<uint64_t> responded_sequences_;
  std::vector<double> pc_to_mcu_samples_ms_;
  std::vector<double> mcu_to_pc_samples_ms_;
  rclcpp::Publisher<std_msgs::msg::UInt64MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::UInt64MultiArray>::SharedPtr
      subscription_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr finish_timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<LatencyTestNode>());
  }
  catch (const std::exception& error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("microros_latency_test"), "%s",
                 error.what());
    rclcpp::shutdown();
    return 1;
  }
  return 0;
}
