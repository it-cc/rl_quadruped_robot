#ifndef SERVO_PWM_H
#define SERVO_PWM_H

#include <tim.h>

namespace servo
{
enum class ServoDirection
{
  Middle = 0,
  Left = 1,
  Right = -1
};
class PwmServo
{
 public:
  PwmServo(TIM_HandleTypeDef* timer, uint32_t channel, ServoDirection direction,
           float servoMinPulseMsValue, float servoMaxPulseMsValue,
           float physicalDegreeRangeValue, float periodMsValue)
      : timer_(timer),
        channel_(channel),
        direction_(direction),
        servoMinPulseMs(servoMinPulseMsValue),
        servoMaxPulseMs(servoMaxPulseMsValue),
        physicalDegreeRange(physicalDegreeRangeValue),
        periodMs(periodMsValue)
  {
  }
  bool start();
  bool stop();
  bool setAngle(float angle);
  bool setPulseWidthMs(float pulse_ms);

  TIM_HandleTypeDef* timer_;
  uint32_t channel_;
  ServoDirection direction_;
  float servoMinPulseMs;
  float servoMaxPulseMs;
  float physicalDegreeRange;
  float periodMs;
};

}  // namespace servo

#endif  // SERVO_PWM_H
