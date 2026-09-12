#ifndef IMU_JY901S_TEST_H
#define IMU_JY901S_TEST_H

#include "api_task.h"
#include "imu_JY901S.h"
#include "rl_servo.h"

namespace test
{
class ImuJY901STest : public task::ManagedTask
{
 public:
  explicit ImuJY901STest(UART_HandleTypeDef& imuUart);

  void init();

 protected:
  void taskProcess() override;

 private:
  enum class State
  {
    Idle,
    WaitingConfig,
    Streaming,
  };

  static constexpr uint32_t kConfigWaitMs = 500;
  static constexpr uint32_t kStreamPeriodMs = 20;

  imu::JY901S imu_;
  rl_controller::RL_Servo servo_;
  State state_{State::Idle};
  uint32_t stateStartTick_{0};
  uint32_t lastStreamTick_{0};
  bool configPrinted_{false};
  bool isCail_{false};

  void printConfig(const imu::Cfg& cfg);
  void printData();
};
}  // namespace test

#endif  // IMU_JY901S_TEST_H
