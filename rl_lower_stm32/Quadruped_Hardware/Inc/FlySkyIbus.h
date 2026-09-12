#ifndef FLY_SKY_IBUS_H
#define FLY_SKY_IBUS_H

#include "api_serial.h"

#include <cstdint>

namespace remote_control
{
class FlySkyIbus : public serial::UartRx
{
 public:
  static constexpr uint8_t kChannelCount = 10;

  explicit FlySkyIbus(UART_HandleTypeDef& huart);
  ~FlySkyIbus() = default;

  void start();
  void update();

  uint16_t getChannel(uint8_t index) const;
  bool isOnline() const;
  int16_t getLeftX() const;
  int16_t getLeftY() const;
  int16_t getRightX() const;
  int16_t getRightY() const;
  uint8_t getSwitchA() const;
  uint8_t getSwitchB() const;
  uint8_t getSwitchC() const;
  uint8_t getSwitchD() const;
  uint32_t getValidFrameCount() const;
  uint32_t getChecksumErrorCount() const;

 protected:
  void OnDataReceived(uint8_t* data, uint16_t length) override;

 private:
  static constexpr uint8_t kFrameLength = 32;
  static constexpr uint16_t kRxBufferLength = 64;
  static constexpr uint32_t kSignalTimeoutMs = 100;
  static constexpr uint16_t kChannelCenter = 1500;
  static constexpr int16_t kStickDeadband = 50;

  uint8_t rxBuffer_[kRxBufferLength]{};
  uint8_t frame_[kFrameLength]{};
  uint8_t frameIndex_{0};
  uint16_t channels_[kChannelCount]{};
  int16_t leftX_{0};
  int16_t leftY_{0};
  int16_t rightX_{0};
  int16_t rightY_{0};
  uint8_t switchA_{0};
  uint8_t switchB_{0};
  uint8_t switchC_{0};
  uint8_t switchD_{0};
  bool online_{false};
  uint32_t validFrameCount_{0};
  uint32_t checksumErrorCount_{0};
  uint32_t lastValidFrameMs_{0};

  void consumeByte(uint8_t byte);
  void processFrame();
  void publishFrame();
  void resetOutputs();
  static int16_t applyDeadband(uint16_t value);
  static uint8_t decodeTwoPositionSwitch(uint16_t value);
  static uint8_t decodeThreePositionSwitch(uint16_t value);
};
}  // namespace remote_control

#endif  // FLY_SKY_IBUS_H
