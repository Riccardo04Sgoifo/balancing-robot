#include "pid.h"

PID::PID(float kp_, float ki_, float kd_){
  kp = kp_;
  ki = ki_;
  kd = kd_;

  prev_error = 0;
  sum_error = 0;
}

void PID::set_gains(float kp_, float ki_, float kd_){
  kp = kp_;
  ki = ki_;
  kd = kd_;
}
 

float PID::compute(float error, float dt) {

  sum_error += error * dt;

  sum_error = constrain(sum_error, -100.0, 100.0);


  float proportional = error * kp;
  
  float integral  = sum_error * ki;

  float derivative = (error - prev_error) / dt * kd;
  prev_error = error;

  float output = proportional + integral + derivative;

  return output;
}