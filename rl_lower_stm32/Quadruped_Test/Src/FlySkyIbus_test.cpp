#include "FlySkyIbus_test.h"

#include "api_serial.h"
#include "stm32h5xx_hal.h"

namespace test
{
namespace
{
constexpr uint8_t kVofaTail[] = {0x00, 0x00, 0x80, 0x7F};

void sendFloat(float value)
{
  HAL_UART_Transmit(&huart2, reinterpret_cast<const uint8_t *>(&value),
                    sizeof(value), 100);
}
}  // namespace

FlySkyIbusTest::FlySkyIbusTest(UART_HandleTypeDef& receiverUart)
    : task::ManagedTask("flysky_ibus_test", 20, 512, task::TaskType::TASK_PERIOD,
                        1),
      receiver_(receiverUart)
{
}

void FlySkyIbusTest::init()
{
  receiver_.start();
  lastPrintTick_ = HAL_GetTick();
}

void FlySkyIbusTest::taskProcess()
{
  receiver_.update();

  const uint32_t now = HAL_GetTick();
  if (static_cast<uint32_t>(now - lastPrintTick_) < kPrintPeriodMs) return;

  lastPrintTick_ = now;
  printData();
}

void FlySkyIbusTest::printData()
{
  if (!receiver_.isOnline()) return;

  for (uint8_t i = 0; i < kVofaChannelCount; ++i)
    sendFloat(static_cast<float>(receiver_.getChannel(i)));

  HAL_UART_Transmit(&huart2, kVofaTail, sizeof(kVofaTail), 100);
}
}  // namespace test
