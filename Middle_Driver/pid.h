#ifndef __MY_PID_H
#define __MY_PID_H
#define STD_KP 1 // 1表示待定
#define STD_KI 1 // 1表示待定
#define STD_KD 1 // 1表示待定
#define INTEGRAL_MAX_LIMIT 1 // 1表示待定
#define INTEGRAL_MIN_LIMIT 1 // 1表示待定
#define PWM_MAX_LIMIT 1799
#define PWM_MIN_LIMIT -1799
#define DEAD_BAND_WIDTH 2 // 误差绝对值<2不再调节

#include <stdint.h>

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Target_Speed;
    float Current_Speed;
    float Current_Error; // Target_Speed - Current_Speed
    float Last_Error;
    float integral;
    float Integral_Max;
    float Integral_Min;
    int32_t PWM_Max;
    int32_t PWM_Min;
    float Dead_Band;
} PID_InitTypeDef;

void PID_Init(PID_InitTypeDef *PID_InitStruct);
void PID_SetPID(PID_InitTypeDef *PID_InitStruct, float New_Kp, float New_Ki, float New_Kd);
void PID_SetTarget_Speed(PID_InitTypeDef *PID_InitStruct, float Target_Speed);
void PID_UpdateCurrent_Speed(PID_InitTypeDef *PID_InitStruct, float Current_Speed);
int32_t PID_Calc(PID_InitTypeDef *PID_InitStruct);
#endif
