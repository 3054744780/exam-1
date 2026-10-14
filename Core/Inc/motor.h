//
// Created by 30547 on 2026/10/8.
//

#ifndef SECOND_MOTOR_H
#define SECOND_MOTOR_H
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "ins.h"
#include "mpu6050.h"
extern  float goal_degree;
extern float speed_ref_1;
extern float Kp1, Ki1, Kd1;
extern float degree_error;
extern float last_roll;
extern float degree_integral;
extern float last_degree_error ;
extern float vary_speed_degree ;
extern float motor_speed ;
extern float target_speed ;
extern float speed_error ;
extern float real_speed;
extern float speed_integral;
extern float vary_speed_speed;
extern float last_speed_error;
extern float speed_ref_2;
extern float Kp2, Ki2, Kd2;
void motor (float x);
#endif //SECOND_MOTOR_H
