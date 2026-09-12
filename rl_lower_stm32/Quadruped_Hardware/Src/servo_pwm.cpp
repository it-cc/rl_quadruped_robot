#include "servo_pwm.h"

#include <algorithm>

namespace servo
{
bool PwmServo::start()
{
  return timer_ && HAL_TIM_PWM_Start(timer_, channel_) == HAL_OK;
}

bool PwmServo::stop()
{
  return timer_ && HAL_TIM_PWM_Stop(timer_, channel_) == HAL_OK;
}

bool PwmServo::setAngle(float angle)
{
  if (timer_ == nullptr || timer_->Instance == nullptr)
  {
    return false;
  }

  const float pulse_ms =
      servoMinPulseMs +
      angle / physicalDegreeRange * (servoMaxPulseMs - servoMinPulseMs);
  return setPulseWidthMs(pulse_ms);
}

bool PwmServo::setPulseWidthMs(float pulse_ms)
{
  if (timer_ == nullptr || timer_->Instance == nullptr)
  {
    return false;
  }

  const float clamped_pulse_ms =
      std::clamp(pulse_ms, servoMinPulseMs, servoMaxPulseMs);
  const uint32_t period_counts = __HAL_TIM_GET_AUTORELOAD(timer_) + 1U;
  const float compare =
      clamped_pulse_ms * static_cast<float>(period_counts) / periodMs;
  __HAL_TIM_SET_COMPARE(timer_, channel_,
                        static_cast<uint32_t>(compare + 0.5F));
  return true;
}

}  // namespace servo
