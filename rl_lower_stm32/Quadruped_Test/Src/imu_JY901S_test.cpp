#include "api_serial.h"
#include "imu_JY901S_test.h"
#include "stm32h5xx_hal.h"
#include "usart.h"
#include <array>
namespace test
{
namespace
{
constexpr uint8_t kVofaTail[] = {0x00, 0x00, 0x80, 0x7F};

constexpr std::array<float, 12> kDefaultJointPosition = {
    0.0F, 0.9F, -1.8F, 0.0F, 0.9F, -1.8F, 0.0F, 0.9F, -1.8F, 0.0F, 0.9F, -1.8F};
void sendFloat(float value)
{
  HAL_UART_Transmit(&huart7, reinterpret_cast<const uint8_t*>(&value),
                    sizeof(value), 100);
}
}  // namespace

ImuJY901STest::ImuJY901STest(UART_HandleTypeDef& imuUart)
    : task::ManagedTask("imu_jy901s_test", 20, 512, task::TaskType::TASK_PERIOD,
                        1),
      imu_(imuUart)
{
}

void ImuJY901STest::init()
{
  imu_.start();
  imu_.readCfg();

  servo_.init();
  servo_.setAllServoAngles(kDefaultJointPosition.data());

  stateStartTick_ = HAL_GetTick();
  state_ = State::WaitingConfig;
  const int result = uart_printf("JY901S config reading, uart=%p\r\n",
                                 static_cast<void*>(&huart2));
  (void)result;
}

void ImuJY901STest::taskProcess()
{
  if (!isCail_)
  {
    isCail_=true;
    imu_.rollPitchCalibration(3000);
    imu_.yawCalibration(3000);
  }
  const uint32_t now = HAL_GetTick();

  if (state_ == State::WaitingConfig)
  {
    if (static_cast<uint32_t>(now - stateStartTick_) < kConfigWaitMs) return;

    if (!configPrinted_)
    {
      printConfig(imu_.readCfg());
      configPrinted_ = true;
    }
    lastStreamTick_ = now;
    state_ = State::Streaming;
    uart_printf("JY901S VOFA stream start\r\n");
    return;
  }

  if (state_ != State::Streaming ||
      static_cast<uint32_t>(now - lastStreamTick_) < kStreamPeriodMs)
    return;

  lastStreamTick_ = now;
  printData();
}

void ImuJY901STest::printConfig(const imu::Cfg& cfg)
{
  uart_printf("JY901S config:\r\n");
  uart_printf("outputRate=%ld\r\n", static_cast<long>(cfg.outputRate));
  uart_printf("content=0x%08lX\r\n", static_cast<unsigned long>(cfg.content));
  uart_printf("bandwidth=%ld\r\n", static_cast<long>(cfg.bandwidth));
  uart_printf("accelerationRange=%.3f g\r\n",
              static_cast<double>(cfg.accelerationRange));
  uart_printf("gyroRange=%.3f deg/s\r\n", static_cast<double>(cfg.gyroRange));
}

void ImuJY901STest::printData()
{
  const types::Euler angle = imu_.getAngle();
  const types::Quaternion quaternion = imu_.getQuaternion();
  const types::Gyroscope gyro = imu_.getAngularVelocity();
  const types::Accelerometer acceleration = imu_.getAcceleration();

  // VOFA+ JustFloat: 13 float channels followed by 00 00 80 7F.
  // lower:pitch(+) raise:pitch(-) left:roll(+) right:roll(-)  左旋:yaw(-) 右旋yaw(+)
  sendFloat(angle.roll);
  sendFloat(angle.pitch);
  sendFloat(angle.yaw);
  sendFloat(gyro.x);
  sendFloat(gyro.y);
  sendFloat(gyro.z);
  sendFloat(acceleration.x);
  sendFloat(acceleration.y);
  sendFloat(acceleration.z);
  HAL_UART_Transmit(&huart7, kVofaTail, sizeof(kVofaTail), 100);
}
}  // namespace test
