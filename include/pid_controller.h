#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <cstdint>

class PidController
{
    public:
        PidController(float kp, float ki, float kd);
        int pid_calculation(float setpoint, float feedback, float delta_time);
        int16_t left_motor_speed(int16_t min_speed, uint16_t max_speed, int16_t motor_bias, int u);
        int16_t right_motor_speed(int16_t min_speed, uint16_t max_speed, int16_t motor_bias, int u);

    private:
        /* VARIAVEIS */
        float _kp;
        float _ki;
        float _kd;
        float _integral;
        float _previous_error;
};

#endif