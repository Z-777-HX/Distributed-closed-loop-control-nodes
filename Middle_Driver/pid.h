#ifndef __PID_H__
#define __PID_H__
#define STD_KP 36.0f
#define STD_KI 1.7f
#define STD_KD 0.0f
#define INTEGRAL_MAX_LIMIT 1800.0f
#define INTEGRAL_MIN_LIMIT -1800.0f
#define PWM_MAX_LIMIT 1799
#define PWM_MIN_LIMIT -1799
#define DEAD_BAND_WIDTH 0.5f // 误差绝对值<0.5不再调节

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
    int16_t PWM_Max;
    int16_t PWM_Min;
    float Dead_Band;
} PID_InitTypeDef;

void PID_Init(PID_InitTypeDef *PID_InitStruct);
void PID_SetPID(PID_InitTypeDef *PID_InitStruct, float New_Kp, float New_Ki, float New_Kd);
void PID_SetTarget_Speed(PID_InitTypeDef *PID_InitStruct, float Target_Speed);
void PID_UpdateCurrent_Speed(PID_InitTypeDef *PID_InitStruct, float Current_Speed);
int16_t PID_Calc(PID_InitTypeDef *PID_InitStruct);

#endif
