#include "PID.h"
#include <math.h>
#include <stdio.h>
#include <sys/_stdint.h>
#include "MadgwickAHRS.h"
#include "stm32f4xx_tim.h"

const float OUTER_INT_MAX = 40.0f;      // 外环积分限幅，对应角度误差累积和
const float INNER_INT_MAX = 40.0f;      // 内环积分限幅，对应角速度误差累积和
const float PWM_OUT_MAX = 200.0f;       // PWM 输出限幅，200.0 对应 20% 的油门

// 对应 roll 内环、 roll 外环、 pitch 内环、 pitch 外环、 yaw 单环
PID_t rollInner, rollOuter, pitchInner, pitchOuter, yawSingle;
// 期望的 roll, pitch, yaw 角速度和油门（即遥控器发出的信号）
float expRoll, expPitch, expYaw, expMode;
// PID 输出的油门控制量（范围在 [-PWM_OUT_MAX, PWM_OUT_MAX]）
float pidRoll, pidPitch, pidYaw;


extern uint16_t data[];
extern Angle angle;
extern float deltaT;

void pidInit() {
    // roll
    rollOuter.Kp = 5.0f;
    rollOuter.Ki = 1.5f;
    rollOuter.Kd = 0.3f;

    rollInner.Kp = 1.5f;
    rollInner.Ki = 0.0f;
    rollInner.Kd = 15.0f;

    // pitch
    pitchOuter.Kp = 5.0f;
    pitchOuter.Ki = 1.5f;
    pitchOuter.Kd = 0.3f;

    pitchInner.Kp = 1.5f;
    pitchInner.Ki = 0.0f;
    pitchInner.Kd = 15.0f;

    // Yaw
    yawSingle.Kp = 10.0f;
    yawSingle.Ki = 0.0f;
    yawSingle.Kd = 4.0f;
}

static float limit(float x, float minv, float maxv) {
    return x < minv ? minv : (x > maxv ? maxv : x);
}

/**
 * @brief 串级 PID 控制
 * 
 * @param errTheta 误差角度
 * @param omega 对应轴的角速度（单位：度/秒）
 * @param outer 外环 PID
 * @param inner 内环 PID
 * @return float 电机输出
 */
float serialPIDcontrol(float errTheta, float omega, PID_t *outer, PID_t *inner) {
    // if (++outer->cnt2 == 2) {
        // if (fabsf(errTheta) > 1.0f) {
            outer->err = errTheta;
            if (expMode > 1440)
                outer->errSum = limit(outer->errSum + outer->err * 0.003, -OUTER_INT_MAX, OUTER_INT_MAX);
            outer->outputP = outer->Kp * outer->err;
            outer->outputI = outer->Ki * outer->errSum;
            outer->outputD = -outer->Kd * omega;
            outer->output = outer->outputP + outer->outputI + outer->outputD;
            // outer->output = outer->Kp * outer->err + outer->Ki * outer->errSum - outer->Kd * omega;
        // }
        // outer->cnt2 = 0;
    // }

    inner->err = outer->output - omega;
    if (expMode > 1440)
        inner->errSum = limit(inner->errSum + inner->err * 0.003, -INNER_INT_MAX, INNER_INT_MAX);
    // inner->output = inner->Kp * inner->err + inner->Ki * inner->errSum + inner->Kd * (inner->err - inner->errLast);
    inner->outputP = inner->Kp * inner->err;
    inner->outputI = inner->Ki * inner->errSum;
    inner->outputD = inner->Kd * (inner->err - inner->errLast);
    // inner->outputD = inner->Kd * (inner->errLast - omega);
    inner->output = inner->outputP + inner->outputI + inner->outputD;
    inner->errLast = inner->err;
    if (++inner->cnt == 10) {
        inner->errLast = inner->err;
        // inner->errLast = omega;
        inner->cnt = 0;
    }
    // printf("outer->output = %f omega = %f errLast = %f err = %f outputD = %f\r\n", outer->output, omega, inner->errLast, inner->err, inner->outputD);
    inner->output = limit(inner->output, -PWM_OUT_MAX, PWM_OUT_MAX);

    return inner->output;
}

float singlePIDcontrol(float omega, PID_t *pid) {
    pid->err = expYaw - omega;
    if (expMode > 1440)
        pid->errSum = limit(pid->errSum + pid->err * 0.003, -OUTER_INT_MAX, OUTER_INT_MAX);
    pid->outputP = pid->Kp * pid->err;
    pid->outputI = pid->Ki * pid->errSum;
    pid->outputD = pid->Kd * (pid->err - pid->errLast);
    if (++pid->cnt == 10) {
        pid->errLast = pid->err;
        pid->cnt = 0;
    }
    pid->output = pid->outputP + pid->outputI + pid->outputD;
    pid->output = limit(pid->output, -PWM_OUT_MAX, PWM_OUT_MAX);

    return pid->output;
}

/**
 * @brief PID 控制入口
 * 
 * @param gyroFilterd 滤波后的陀螺仪数据（单位：弧度/秒）
 */
void pidControl(float *gyroFilterd) {
    float motor1, motor2, motor3, motor4;

    // 接收机信号解析，范围均在 [1000, 2000]
    expRoll = limit((data[4] - 1500) * 0.02f, -10, 10);   // roll 在 [-10, 10] 度
    expPitch = limit(-(data[2] - 1500) * 0.02f, -10, 10); // pitch 在 [-10, 10] 度
    expYaw = limit(-(data[1] - 1500) * 0.1f, -50, 50);    // yaw 在 [-50, 50] 度/秒
    expMode = limit(data[3], 1000, 1800);                 // 油门在 [1000, 1800]

    // 串级 PID 控制
    // printf("expRoll = %f angle.roll = %f\r\n", expRoll, angle.roll);
    pidRoll = serialPIDcontrol(expRoll - angle.roll, gyroFilterd[0] * RAD_TO_DEGREE, &rollOuter, &rollInner);
    pidPitch = serialPIDcontrol(expPitch - angle.pitch, gyroFilterd[1] * RAD_TO_DEGREE, &pitchOuter, &pitchInner);
    pidYaw = singlePIDcontrol(gyroFilterd[2] * RAD_TO_DEGREE, &yawSingle);
    

    // 电机输出（1000 对应 0% 油门，2000 对应 100% 油门）
    // motor1 = limit(expMode + pidRoll - pidPitch, 1000, 2000);
    // motor2 = limit(expMode - pidRoll - pidPitch, 1000, 2000);
    // motor3 = limit(expMode + pidRoll + pidPitch, 1000, 2000);
    // motor4 = limit(expMode - pidRoll + pidPitch, 1000, 2000);

    // 电机输出（1000 对应 0% 油门，2000 对应 100% 油门）
    // motor1 = limit(expMode + 1.1f * pidRoll - pidPitch - pidYaw, 1000, 1800);
    // motor2 = limit(expMode + 1.1f * pidRoll + pidPitch + pidYaw, 1000, 1800);
    // motor3 = limit(expMode - 0.9f * pidRoll - pidPitch + pidYaw, 1000, 1800);
    // motor4 = limit(expMode - 0.9f * pidRoll + pidPitch - pidYaw, 1000, 1800);
    motor1 = limit(expMode + pidRoll - pidPitch - pidYaw, 1000, 1800);
    motor2 = limit(expMode + pidRoll + pidPitch + pidYaw, 1000, 1800);
    motor3 = limit(expMode - pidRoll - pidPitch + pidYaw, 1000, 1800);
    motor4 = limit(expMode - pidRoll + pidPitch - pidYaw, 1000, 1800);

    // 通道 7 用于遥控器加锁与解锁
    if (data[7] > 1500) {
        TIM_SetCompare1(TIM3, motor1);
        TIM_SetCompare2(TIM3, motor2);
        TIM_SetCompare3(TIM3, motor3);
        TIM_SetCompare4(TIM3, motor4);
    } else {
        TIM_SetCompare1(TIM3, 1000);
        TIM_SetCompare2(TIM3, 1000);
        TIM_SetCompare3(TIM3, 1000);
        TIM_SetCompare4(TIM3, 1000);
    }
}
