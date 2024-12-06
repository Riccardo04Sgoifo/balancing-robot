#include "Arduino.h"

#pragma once

class PID {
public:
  PID(float kp_, float ki_, float kd_);
  void set_gains(float kp_, float ki_, float kd_);

  void set_p_gain(float kp_) { kp = kp_; }
  void set_i_gain(float ki_) { ki = ki_; }
  void set_d_gain(float kd_) { kd = kd_; }

  float compute(float error, float dt);

private:
  float kp; // Proportional gain
  float ki; // Integral gain
  float kd; // Derivative gain
  float dt; // Time step
  float prev_error; // Previous error value
  float sum_error; // Integral value
};