//
// Created by 30547 on 2026/10/8.
//

#include "motor.h"
void motor (float x)
{

    //方向与刹车
    if (x > 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    }
    if (x < 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    }
    if (x == 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    }

    //赋予速度
    if (x >= 0)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, x);
    else __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -x);
}