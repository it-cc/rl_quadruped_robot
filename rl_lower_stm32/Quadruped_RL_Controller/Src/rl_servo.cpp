#include "rl_servo.h"

#include "api_serial.h"

namespace rl_controller
{
namespace
{
constexpr float kRadiansToDegrees = 57.2957795131F;

// Joint order: FL (0..2), FR (3..5), BL (6..8), BR (9..11).
// Front hip signs are kept unchanged; rear hips, all thighs, and all calves
// use the opposite sign from the policy joint coordinate.
constexpr bool kInvertPolicySign[12] = {
    false, true, true,
    false, true, true,
    false, true, true,
    false, true, true};
}

void RL_Servo::init()
{
  for (int index = 0; index < 12; ++index)
  {
    const bool started = servos_[index].start();
    const bool positioned = servos_[index].setAngle(offsets_[index]);
    if (!started || !positioned)
    {
      uart_printf("[SERVO] init failed index=%d start=%d position=%d\r\n",
                  index, started ? 1 : 0, positioned ? 1 : 0);
    }
  }
}

void RL_Servo::setAllServoAngles(const float angles[12])
{
  if (angles == nullptr) return;
  for (int index = 0; index < 12; ++index)
  {
    // ROS joint commands are radians; the calibrated PWM model uses degrees.
    float angle = angles[index] * kRadiansToDegrees;
    if (kInvertPolicySign[index]) angle = -angle;
    if (angle > limits_[index].max) angle = limits_[index].max;
    if (angle < limits_[index].min) angle = limits_[index].min;

    switch (servos_[index].direction_)
    {
      case servo::ServoDirection::Left:
        angle = offsets_[index] - angle;
        break;
      case servo::ServoDirection::Right:
        angle = offsets_[index] + angle;
        break;
      case servo::ServoDirection::Middle:
        angle = offsets_[index] - angle;
        break;
    }
    if (!servos_[index].setAngle(angle))
    {
      uart_printf("[SERVO] command failed index=%d angle_deg=%.2f\r\n", index,
                  static_cast<double>(angle));
    }
  }
}
}  // namespace rl_controller
