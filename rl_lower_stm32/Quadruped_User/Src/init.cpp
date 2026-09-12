#include "init.h"
#include "imu_JY901S_test.h"
#include "rl_microros_interface.h"
#include "usart.h"

namespace
{
rl_controller::RL_MicroRosInterface rlHardware(huart2, huart3, huart5);
// test::ImuJY901STest imuTest(huart3);
}  // namespace

extern "C" void allInit()
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  rlHardware.init();
  // imuTest.init();
}
