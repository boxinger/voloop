#ifndef __PID_H
#define __PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;
} PID_InitTypeDef;

typedef enum {
    PID_ERROR = 0U, // Reserved
    PID_UnSaturated,
    PID_UpperSaturated,
    PID_LowerSaturated
} PID_StateTypeDef;

typedef struct PID_HandleTypeDef PID_HandleTypeDef;

PID_HandleTypeDef* PID_Init(PID_InitTypeDef* init);
void PID_DeInit(PID_HandleTypeDef* handle);
void PID_Reset(PID_HandleTypeDef* handle);
void PID_SetIntegral(PID_HandleTypeDef* handle, float integral) ;
void PID_SetPreviousError(PID_HandleTypeDef* handle, float previousError);
PID_StateTypeDef PID_GetState(PID_HandleTypeDef* handle);

float PID_Compute(PID_HandleTypeDef* handle, float setpoint, float measurement);

// 条件积分法：在输出饱和且误差继续推动同方向饱和时暂停积分
float PID_ComputeConditional(PID_HandleTypeDef* handle,
                             float setpoint,
                             float measurement,
                             float outputMin,
                             float outputMax);

// 反算回注法：通过 Kaw * (u_sat - u_raw) 抑制积分饱和
float PID_ComputeBackCalculation(PID_HandleTypeDef* handle,
                                 float setpoint,
                                 float measurement,
                                 float outputMin,
                                 float outputMax,
                                 float antiWindupGain);


#endif /* __PID_H */
