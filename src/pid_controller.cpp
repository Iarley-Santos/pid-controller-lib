#include "pid_controller.h"

pidController::pidController(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _integral = 0.0f;
    _previous_error = 0.0f;
}

int PidController::pid_calculation(float setpoint, float feedback, float delta_time)
{
    float error = setpoint - feedback;
    
    _integral += error * delta_time;
    float derivative = (delta_time > 0.0f) ? (error - _previous_error) / delta_time : 0.0f;
    
    float output = (_kp * error) + (_ki * _integral) + (_kd * derivative);

    _previous_error = error;
    int u = static_cast<int>(output);
    return u;
}

int16_t PidController::right_motor_speed(int16_t min_speed, uint16_t max_speed, int16_t motor_bias, int u)
{
    int16_t speed = motor_bias + u;
    if (speed > max_speed) speed = max_speed;
    if (speed < min_speed) speed = min_speed;
    return speed;
}

int16_t PidController::left_motor_speed(int16_t min_speed, uint16_t max_speed, int16_t motor_bias, int u)
{
    int16_t speed = motor_bias - u;
    if (speed > max_speed) speed = max_speed;
    if (speed < min_speed) speed = min_speed;
    return speed;
}