#include "FlySkyIbus.h"

namespace remote_control
{
namespace
{
constexpr uint8_t kFrameHeader = 0x20;
constexpr uint8_t kFrameType = 0x40;
}  // namespace

FlySkyIbus::FlySkyIbus(UART_HandleTypeDef& huart)
    : serial::UartRx(huart, rxBuffer_, kRxBufferLength, true, true, false)
{
  for (uint8_t i = 0; i < kChannelCount; ++i) channels_[i] = kChannelCenter;
}

void FlySkyIbus::start()
{
  frameIndex_ = 0;
  online_ = false;
  resetOutputs();
  serial::UartRx::start();
}

void FlySkyIbus::update()
{
  online_ = validFrameCount_ != 0 &&
            (HAL_GetTick() - lastValidFrameMs_) <= kSignalTimeoutMs;
  if (!online_)
  {
    resetOutputs();
    return;
  }

  rightX_ = applyDeadband(channels_[0]);
  rightY_ = applyDeadband(channels_[1]);
  leftY_ = applyDeadband(channels_[2]);
  leftX_ = applyDeadband(channels_[3]);
  switchA_ = decodeTwoPositionSwitch(channels_[4]);
  switchB_ = decodeThreePositionSwitch(channels_[7]);
  switchC_ = decodeThreePositionSwitch(channels_[5]);
  switchD_ = decodeTwoPositionSwitch(channels_[6]);
}

void FlySkyIbus::OnDataReceived(uint8_t* data, uint16_t length)
{
  if (data == nullptr) return;
  for (uint16_t i = 0; i < length; ++i) consumeByte(data[i]);
}

void FlySkyIbus::consumeByte(uint8_t byte)
{
  if (frameIndex_ == 0)
  {
    if (byte == kFrameHeader) frame_[frameIndex_++] = byte;
    return;
  }

  if (frameIndex_ == 1 && byte != kFrameType)
  {
    frameIndex_ = (byte == kFrameHeader) ? 1 : 0;
    if (frameIndex_ == 1) frame_[0] = byte;
    return;
  }

  frame_[frameIndex_++] = byte;
  if (frameIndex_ != kFrameLength) return;

  processFrame();
  frameIndex_ = 0;
}

void FlySkyIbus::processFrame()
{
  uint16_t checksum = 0xFFFF;
  for (uint8_t i = 0; i < kFrameLength - 2; ++i)
    checksum = static_cast<uint16_t>(checksum - frame_[i]);

  const uint16_t receivedChecksum = static_cast<uint16_t>(frame_[30]) |
                                    (static_cast<uint16_t>(frame_[31]) << 8);
  if (checksum != receivedChecksum)
  {
    ++checksumErrorCount_;
    return;
  }
  publishFrame();
}

void FlySkyIbus::publishFrame()
{
  for (uint8_t i = 0; i < kChannelCount; ++i)
  {
    const uint8_t offset = static_cast<uint8_t>(2 + i * 2);
    channels_[i] = static_cast<uint16_t>(frame_[offset]) |
                   (static_cast<uint16_t>(frame_[offset + 1]) << 8);
  }
  lastValidFrameMs_ = HAL_GetTick();
  ++validFrameCount_;
  online_ = true;
}

void FlySkyIbus::resetOutputs()
{
  leftX_ = 0;
  leftY_ = 0;
  rightX_ = 0;
  rightY_ = 0;
  switchA_ = 0;
  switchB_ = 0;
  switchC_ = 0;
  switchD_ = 0;
}

int16_t FlySkyIbus::applyDeadband(uint16_t value)
{
  const int16_t error = static_cast<int16_t>(value) - kChannelCenter;
  if (error > kStickDeadband) return error - kStickDeadband;
  if (error < -kStickDeadband) return error + kStickDeadband;
  return 0;
}

uint8_t FlySkyIbus::decodeTwoPositionSwitch(uint16_t value)
{
  return value > kChannelCenter ? 1 : 0;
}

uint8_t FlySkyIbus::decodeThreePositionSwitch(uint16_t value)
{
  if (value <= 1250) return 0;
  if (value <= 1750) return 1;
  return 2;
}

uint16_t FlySkyIbus::getChannel(uint8_t index) const
{
  return index < kChannelCount ? channels_[index] : 0;
}

bool FlySkyIbus::isOnline() const { return online_; }
int16_t FlySkyIbus::getLeftX() const { return leftX_; }
int16_t FlySkyIbus::getLeftY() const { return leftY_; }
int16_t FlySkyIbus::getRightX() const { return rightX_; }
int16_t FlySkyIbus::getRightY() const { return rightY_; }
uint8_t FlySkyIbus::getSwitchA() const { return switchA_; }
uint8_t FlySkyIbus::getSwitchB() const { return switchB_; }
uint8_t FlySkyIbus::getSwitchC() const { return switchC_; }
uint8_t FlySkyIbus::getSwitchD() const { return switchD_; }
uint32_t FlySkyIbus::getValidFrameCount() const { return validFrameCount_; }
uint32_t FlySkyIbus::getChecksumErrorCount() const
{
  return checksumErrorCount_;
}

}  // namespace remote_control
