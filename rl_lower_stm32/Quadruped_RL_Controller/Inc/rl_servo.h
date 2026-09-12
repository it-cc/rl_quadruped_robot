#ifndef RL_SERVO_H
#define RL_SERVO_H

#ifdef __cplusplus

#include <tim.h>

#include "servo_pwm.h"

namespace rl_controller
{
class RL_Servo
{
 public:
  void init();
  void setAllServoAngles(const float angles[12]);

 private:
  struct Limit
  {
    float min;
    float max;
  };

  servo::PwmServo servos_[12]{
      {&htim3, TIM_CHANNEL_1, servo::ServoDirection::Right, 0.5f, 2.5f, 270.0f,
       20.0f},
      {&htim2, TIM_CHANNEL_2, servo::ServoDirection::Left, 0.5f, 2.5f, 270.0f,
       20.0f},
      {&htim2, TIM_CHANNEL_3, servo::ServoDirection::Middle, 0.5f, 2.5f,
       270.0f, 20.0f},
      {&htim2, TIM_CHANNEL_4, servo::ServoDirection::Right, 0.5f, 2.5f, 270.0f,
       20.0f},
      {&htim4, TIM_CHANNEL_1, servo::ServoDirection::Right, 0.5f, 2.5f, 270.0f,
       20.0f},
      {&htim4, TIM_CHANNEL_2, servo::ServoDirection::Middle, 0.5f, 2.5f,
       270.0f, 20.0f},
      {&htim4, TIM_CHANNEL_3, servo::ServoDirection::Left, 0.5f, 2.5f, 270.0f,
       20.0f},
      {&htim4, TIM_CHANNEL_4, servo::ServoDirection::Left, 0.5f, 2.5f, 270.0f,
       20.0f},
      {&htim3, TIM_CHANNEL_3, servo::ServoDirection::Middle, 0.5f, 2.5f,
       270.0f, 20.0f},
      {&htim3, TIM_CHANNEL_4, servo::ServoDirection::Left, 0.5f, 2.5f, 270.0f,
       20.0f},
      {&htim8, TIM_CHANNEL_3, servo::ServoDirection::Right, 0.5f, 2.5f, 270.0f,
       20.0f},
      {&htim8, TIM_CHANNEL_4, servo::ServoDirection::Middle, 0.5f, 2.5f,
       270.0f, 20.0f}};

  Limit limits_[12] = {{-45.0f, 45.0f},  {-56.0f, 56.0f},
                       {-30.0f, 126.0f}, {-45.0f, 45.0f},
                       {-56.0f, 56.0f},  {-30.0f, 126.0f},
                       {-45.0f, 45.0f},  {-56.0f, 56.0f},
                       {-30.0f, 126.0f}, {-45.0f, 45.0f},
                       {-56.0f, 56.0f},  {-30.0f, 126.0f}};
  float offsets_[12] = {160.0f, 175.0f, 218.0f, 155.0f, 170.0f, 250.0f,
                        190.0f, 180.0f, 240.0f, 170.0f, 183.0f, 245.0f};
};
}  // namespace rl_controller

#endif  // __cplusplus
#endif  // RL_SERVO_H
