#ifndef TYPES_H
#define TYPES_H

#ifdef __cplusplus

namespace types
{
class Linear
{
 public:
  float x;
  float y;
  float z;
  Linear() : x(0.0f), y(0.0f), z(0.0f) {}
};

class Angular
{
 public:
  float x;
  float y;
  float z;
  Angular() : x(0.0f), y(0.0f), z(0.0f) {}
};

class Velocities
{
 public:
  Linear linear;
  Angular angular;
};

class Quaternion
{
 public:
  Quaternion() : x(0.0f), y(0.0f), z(0.0f), w(0.0f) {}
  float x;
  float y;
  float z;
  float w;
};

class Point
{
 public:
  float x;
  float y;
  float z;
  Point() : x(0.0f), y(0.0f), z(0.0f) {}
};

class Euler
{
 public:
  Euler() : roll(0.0f), pitch(0.0f), yaw(0.0f) {}
  float roll;
  float pitch;
  float yaw;
};

class Pose
{
 public:
  Point position;
  Euler orientation;
};

class Command
{
 public:
  Velocities velocity;
  Pose pose;
};

class Accelerometer
{
 public:
  Accelerometer() : x(0.0f), y(0.0f), z(0.0f) {}
  float x;
  float y;
  float z;
};

class Gyroscope
{
 public:
  Gyroscope() : x(0.0f), y(0.0f), z(0.0f) {}
  float x;
  float y;
  float z;
};

class Magnetometer
{
 public:
  Magnetometer() : x(0.0f), y(0.0f), z(0.0f) {}
  float x;
  float y;
  float z;
};

}  // namespace types

#endif  // __cplusplus
#endif  // TYPES_H
