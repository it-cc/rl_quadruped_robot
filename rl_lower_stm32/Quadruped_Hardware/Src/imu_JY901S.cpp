#include "imu_JY901S.h"

#include "cmsis_os.h"
#include "main.h"

namespace imu
{
namespace
{
constexpr uint8_t kHeader = 0x55;
constexpr uint8_t kRegisterResponse = 0x5F;
constexpr uint8_t kWriteCommand[] = {0xFF, 0xAA};
constexpr uint8_t kReadCommand[] = {0xFF, 0xAA, 0x27};
constexpr uint8_t kKeyRegister = 0x69;
constexpr uint8_t kCalibrationRegister = 0x01;
constexpr uint16_t kUnlockKey = 0xB588;
constexpr uint16_t kNormalCalibration = 0x0000;
constexpr uint16_t kGyroAccCalibration = 0x0001;
constexpr uint16_t kMagneticCalibration = 0x0007;
constexpr uint16_t kYawCalibration = 0x0004;
constexpr uint16_t kRollPitchCalibration = 0x0008;
constexpr uint8_t kSaveRegister = 0x00;
constexpr uint16_t kSaveParameters = 0x0000;
constexpr uint8_t kOutputRateRegister = 0x03;
constexpr uint8_t kContentRegister = 0x02;
constexpr uint8_t kBandwidthRegister = 0x1F;
constexpr uint8_t kAccelerationRangeRegister = 0x21;
constexpr uint8_t kGyroRangeRegister = 0x20;
constexpr float kInt16Scale = 32768.0f;

constexpr uint8_t kAccType = 0x51;
constexpr uint8_t kGyroType = 0x52;
constexpr uint8_t kAngleType = 0x53;
constexpr uint8_t kQuaternionType = 0x59;

float scale(int16_t value, float range)
{
  return static_cast<float>(value) * range / kInt16Scale;
}

float accelerationRange(int16_t value)
{
  switch (value)
  {
    case 0:
      return 2.0f;
    case 1:
      return 4.0f;
    case 2:
      return 8.0f;
    default:
      return 16.0f;
  }
}

float gyroRange(int16_t value)
{
  switch (value)
  {
    case 0:
      return 250.0f;
    case 1:
      return 500.0f;
    case 2:
      return 1000.0f;
    default:
      return 2000.0f;
  }
}
}  // namespace

JY901S::JY901S(UART_HandleTypeDef& huart)
    : serial::UartRx(huart, rxBuffer, JY901S_RX_BUFFER_SIZE, true, true, false),
      huart_(&huart)
{
  resetParser();
}

JY901S::JY901S(UART_HandleTypeDef& huart, Cfg cfg)
    : serial::UartRx(huart, rxBuffer, JY901S_RX_BUFFER_SIZE, true, true, false),
      huart_(&huart),
      cfg_(cfg)
{
  resetParser();
}

JY901S::~JY901S() = default;

void JY901S::resetParser()
{
  rxState_ = JY901SRxState::WaitHeader;
  frameIndex_ = 0;
}

void JY901S::OnDataReceived(uint8_t* buf_, uint16_t len_)
{
  if (buf_ == nullptr) return;
  for (uint16_t i = 0; i < len_; ++i) consumeByte(buf_[i]);
}

void JY901S::consumeByte(uint8_t byte)
{
  switch (rxState_)
  {
    case JY901SRxState::WaitHeader:
      if (byte == kHeader)
      {
        frame_[0] = byte;
        frameIndex_ = 1;
        rxState_ = JY901SRxState::WaitType;
      }
      break;
    case JY901SRxState::WaitType:
      frame_[frameIndex_++] = byte;
      rxState_ = JY901SRxState::WaitPayload;
      break;
    case JY901SRxState::WaitPayload:
      frame_[frameIndex_++] = byte;
      if (frameIndex_ == kFrameSize - 1) rxState_ = JY901SRxState::WaitChecksum;
      break;
    case JY901SRxState::WaitChecksum:
      frame_[frameIndex_++] = byte;
      processFrame();
      resetParser();
      if (byte == kHeader)
      {
        frame_[0] = byte;
        frameIndex_ = 1;
        rxState_ = JY901SRxState::WaitType;
      }
      break;
  }
}

void JY901S::processFrame()
{
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < kFrameSize - 1; ++i) checksum += frame_[i];
  if (checksum != frame_[kFrameSize - 1]) return;

  int16_t values[4]{};
  for (uint8_t i = 0; i < 4; ++i)
  {
    values[i] =
        static_cast<int16_t>(static_cast<uint16_t>(frame_[2 + i * 2]) |
                             (static_cast<uint16_t>(frame_[3 + i * 2]) << 8));
  }

  if (frame_[1] == kRegisterResponse)
    updateConfig(values);
  else
    updateData(frame_[1], values);
}

void JY901S::updateData(uint8_t type, const int16_t values[4])
{
  switch (type)
  {
    case kAccType:
      data_.acceleration.x =
          scale(values[0],
                cfg_.accelerationRange > 0.0f ? cfg_.accelerationRange : 16.0f);
      data_.acceleration.y =
          scale(values[1],
                cfg_.accelerationRange > 0.0f ? cfg_.accelerationRange : 16.0f);
      data_.acceleration.z =
          scale(values[2],
                cfg_.accelerationRange > 0.0f ? cfg_.accelerationRange : 16.0f);
      break;
    case kGyroType:
      data_.angularVelocity.x =
          scale(values[0], cfg_.gyroRange > 0.0f ? cfg_.gyroRange : 2000.0f);
      data_.angularVelocity.y =
          scale(values[1], cfg_.gyroRange > 0.0f ? cfg_.gyroRange : 2000.0f);
      data_.angularVelocity.z =
          scale(values[2], cfg_.gyroRange > 0.0f ? cfg_.gyroRange : 2000.0f);
      break;
    case kAngleType:
      data_.angle.roll = scale(values[0], 180.0f);
      data_.angle.pitch = scale(values[1], 180.0f);
      data_.angle.yaw = scale(values[2], 180.0f);
      break;
    case kQuaternionType:
      data_.quaternion.w = scale(values[0], 1.0f);
      data_.quaternion.x = scale(values[1], 1.0f);
      data_.quaternion.y = scale(values[2], 1.0f);
      data_.quaternion.z = scale(values[3], 1.0f);
      break;
    default:
      break;
  }
}

void JY901S::updateConfig(const int16_t values[4])
{
  if (pendingConfigCount_ == 0) return;

  const uint8_t reg = pendingConfigRegisters_[0];
  for (uint8_t i = 1; i < pendingConfigCount_; ++i)
    pendingConfigRegisters_[i - 1] = pendingConfigRegisters_[i];
  --pendingConfigCount_;

  switch (reg)
  {
    case kOutputRateRegister:
      cfg_.outputRate = values[0];
      break;
    case kContentRegister:
      cfg_.content = values[0];
      break;
    case kBandwidthRegister:
      cfg_.bandwidth = values[0];
      break;
    case kAccelerationRangeRegister:
      cfg_.accelerationRange = accelerationRange(values[0]);
      break;
    case kGyroRangeRegister:
      cfg_.gyroRange = gyroRange(values[0]);
      break;
    default:
      break;
  }

  // A normal-protocol read response contains four consecutive registers.
  // Send the next request only after the previous response was received.
  if (pendingConfigCount_ > 0) readRegister(pendingConfigRegisters_[0]);
}

types::Euler JY901S::getAngle() { return data_.angle; }
types::Quaternion JY901S::getQuaternion() { return data_.quaternion; }
types::Gyroscope JY901S::getAngularVelocity() { return data_.angularVelocity; }
types::Accelerometer JY901S::getAcceleration()
{
  return data_.acceleration;
}

Cfg JY901S::readCfg()
{
  // Do not overlap requests. The returned value is the latest completed cache.
  if (pendingConfigCount_ != 0) return cfg_;

  pendingConfigCount_ = 0;
  const uint8_t registers[] = {kOutputRateRegister, kContentRegister,
                               kBandwidthRegister, kAccelerationRangeRegister,
                               kGyroRangeRegister};
  for (uint8_t reg : registers)
    pendingConfigRegisters_[pendingConfigCount_++] = reg;

  readRegister(pendingConfigRegisters_[0]);
  return cfg_;
}

void JY901S::yawCalibration(uint32_t delay_ms)
{
  // Keep the IMU stationary and point its Z axis in the desired zero heading.
  writeRegister(kKeyRegister, kUnlockKey);
  HAL_Delay(1);
  writeRegister(kCalibrationRegister, kYawCalibration);

  HAL_Delay(delay_ms);

  writeRegister(kCalibrationRegister, kNormalCalibration);
  HAL_Delay(1);
  writeRegister(kSaveRegister, kSaveParameters);
}

void JY901S::rollPitchCalibration(uint32_t delay_ms)
{
  // Place the IMU in the desired reference attitude and keep it stationary.
  writeRegister(kKeyRegister, kUnlockKey);
  HAL_Delay(1);
  writeRegister(kCalibrationRegister, kRollPitchCalibration);

  HAL_Delay(delay_ms);

  writeRegister(kCalibrationRegister, kNormalCalibration);
  HAL_Delay(1);
  writeRegister(kSaveRegister, kSaveParameters);
}

void JY901S::accelerationCalibration(uint32_t delay_ms)
{
  writeRegister(kKeyRegister, kUnlockKey);
  HAL_Delay(1);
  writeRegister(kCalibrationRegister, kGyroAccCalibration);

  HAL_Delay(delay_ms);

  writeRegister(kCalibrationRegister, kNormalCalibration);
  HAL_Delay(1);
  writeRegister(kSaveRegister, kSaveParameters);
}

void JY901S::magneticCalibration(uint32_t delay_ms)
{
  writeRegister(kKeyRegister, kUnlockKey);
  HAL_Delay(1);
  writeRegister(kCalibrationRegister, kMagneticCalibration);

  HAL_Delay(delay_ms);
  writeRegister(kKeyRegister, kUnlockKey);
  HAL_Delay(1);
  writeRegister(kCalibrationRegister, kNormalCalibration);
  HAL_Delay(1);
  writeRegister(kSaveRegister, kSaveParameters);
}

void JY901S::sendBytes(const uint8_t* data, uint16_t length)
{
  if (huart_ == nullptr || data == nullptr || length == 0) return;
  HAL_UART_Transmit(huart_, data, length, 100);
}

void JY901S::writeRegister(uint8_t reg, uint16_t value)
{
  uint8_t frame[5] = {kWriteCommand[0], kWriteCommand[1], reg,
                      static_cast<uint8_t>(value),
                      static_cast<uint8_t>(value >> 8)};
  sendBytes(frame, sizeof(frame));
}

void JY901S::readRegister(uint8_t reg)
{
  uint8_t frame[5] = {kReadCommand[0], kReadCommand[1], kReadCommand[2], reg,
                      0};
  sendBytes(frame, sizeof(frame));
}

}  // namespace imu
