//
// Created by 30547 on 2026/10/9.
//

#include "../Inc/PID_degree.h"
float PID_degree(float z)
{
    ins_data_t *ins = ins_data();
    degree_error = ins->roll - z;
    if (degree_error > 5 || degree_error < -5)
        degree_integral = degree_integral + degree_error * 0.05;
    else degree_integral = 0;
    if (degree_integral > 100)
        degree_integral = 100;
    if (degree_integral < -100)
        degree_integral = -100;
    vary_speed_degree = last_degree_error - degree_error;
    target_speed = Kp1 * degree_error + Ki1 * degree_integral - Kd1 * vary_speed_degree;
    last_degree_error = degree_error;
    return target_speed * 1.8;
}