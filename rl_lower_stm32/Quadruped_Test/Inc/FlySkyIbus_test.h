#ifndef FLY_SKY_IBUS_TEST_H
#define FLY_SKY_IBUS_TEST_H

#include "FlySkyIbus.h"
#include "api_task.h"

namespace test
{
class FlySkyIbusTest : public task::ManagedTask
{
 public:
  explicit FlySkyIbusTest(UART_HandleTypeDef& receiverUart);

  void init();

 protected:
  void taskProcess() override;

 private:
  static constexpr uint8_t kVofaChannelCount = 10;
  static constexpr uint32_t kPrintPeriodMs = 100;

  remote_control::FlySkyIbus receiver_;
  uint32_t lastPrintTick_{0};

  void printData();
};
}  // namespace test

#endif  // FLY_SKY_IBUS_TEST_H
