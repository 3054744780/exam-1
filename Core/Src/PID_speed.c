//
// Created by 30547 on 2026/10/8.
//

#include "PID_speed.h"
float PID_speed(float y)
{
    ins_data_t *ins = ins_data();
    speed_error = y - real_speed;
    speed_integral = speed_integral + speed_error * 0.05;
    if (speed_integral > 80)
        speed_integral = 80;
    if (speed_integral < -80)
        speed_integral = -80;
    if (degree_error < 5 && degree_error > -5)
        speed_integral = 0;
    vary_speed_speed = speed_error - last_speed_error;
    motor_speed = Kp2 * speed_error + Ki2 * speed_integral - Kd2 * vary_speed_speed;
    last_speed_error = speed_error;
    //没什么用的补丁，但是不舍得删
    // if (motor_speed > 60)
    //     motor_speed = 60;
    // if (degree_error < 15)
    //     if (target_speed > 40)
    //         target_speed = 0;
    // if (goal_degree < ins->roll && real_speed == 0 && target_speed < 37 && target_speed > 6)
    //     motor_speed = 37;
    // if (goal_degree > ins->roll && real_speed == 0 && target_speed > -40 && target_speed < -6)
    //     motor_speed = -40;

    //刹车
    if (goal_degree - 6 < ins->roll && ins->roll < goal_degree + 6 && (real_speed > 30 || real_speed < -30))
    {
        motor_speed = 0;
        degree_integral = 0;
        speed_integral = 0;
    }
    return motor_speed;
}