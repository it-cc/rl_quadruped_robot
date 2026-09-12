#ifndef IMU_JY901S_H
#define IMU_JY901S_H

#include "api_serial.h"
#include "types.h"
#define JY901S_RX_BUFFER_SIZE 50

#ifdef __cplusplus

namespace imu
{
struct Data
{
  types::Euler angle;
  types::Quaternion quaternion;
  types::Gyroscope angularVelocity;
  types::Accelerometer acceleration;
};

struct Cfg
{
  int32_t outputRate;
  int32_t content;
  int32_t bandwidth;
  float accelerationRange;  // 加速度计的量程，单位g
  float gyroRange;          // 陀螺仪的量程，单位°/s
};

enum class JY901SRxState
{
  WaitHeader,
  WaitType,
  WaitPayload,
  WaitChecksum,
};

class JY901S : public serial::UartRx
{
 public:
  JY901S(UART_HandleTypeDef& huart);
  JY901S(UART_HandleTypeDef& huart, Cfg cfg);
  ~JY901S();

  types::Euler getAngle();
  types::Quaternion getQuaternion();
  types::Gyroscope getAngularVelocity();
  types::Accelerometer getAcceleration();
  Cfg readCfg();  // 读取imu的配置，需要读取实际cfg

  void yawCalibration(uint32_t delay_ms);           // Z轴归零
  void rollPitchCalibration(uint32_t delay_ms);     // 角度参考
  void accelerationCalibration(uint32_t delay_ms);  // 加速度校准
  void magneticCalibration(uint32_t delay_ms);      // 磁场校准

 protected:
 private:
  uint8_t rxBuffer[JY901S_RX_BUFFER_SIZE];
  static constexpr uint8_t kFrameSize = 11;
  static constexpr uint8_t kConfigRequestCount = 5;

  UART_HandleTypeDef* huart_{nullptr};
  JY901SRxState rxState_{JY901SRxState::WaitHeader};
  uint8_t frame_[kFrameSize]{};
  uint8_t frameIndex_{0};
  Data data_{};
  Cfg cfg_{};
  uint8_t pendingConfigRegisters_[kConfigRequestCount]{};
  uint8_t pendingConfigCount_{0};

  void resetParser();
  void consumeByte(uint8_t byte);
  void processFrame();
  void updateData(uint8_t type, const int16_t values[4]);
  void updateConfig(const int16_t values[4]);
  void sendBytes(const uint8_t* data, uint16_t length);
  void writeRegister(uint8_t reg, uint16_t value);
  void readRegister(uint8_t reg);
  void OnDataReceived(uint8_t* buf_, uint16_t len_) override;
};
}  // namespace imu
#endif

#endif  // IMU_JY901S_H
