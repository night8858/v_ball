#ifndef CHASSIS_H
#define CHASSIS_H

#include "struct_typedef.h"
#include "can_recv.h"
#include "pid.h"

#define PI 3.141592653824f

#define M3505_MOTOR_SPEED_PID_KP 0.0f
#define M3505_MOTOR_SPEED_PID_KI 0.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f



#define M6020_MOTOR_SPEED_PID_KP 0.0f
#define M6020_MOTOR_SPEED_PID_KI 0.0f
#define M6020_MOTOR_SPEED_PID_KD 0.0f
#define M6020_MOTOR_SPEED_PID_MAX_OUT 0.0f
#define M6020_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

#define M6020_MOTOR_POSION_PID_KP 0.0f
#define M6020_MOTOR_POSION_PID_KI 0.0f
#define M6020_MOTOR_POSION_PID_KD 0.0f
#define M6020_MOTOR_POSION_PID_MAX_OUT 0.0f
#define M6020_MOTOR_POSION_PID_MAX_IOUT 2000.0f


#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192

typedef struct
{
    float kp;
    float ki;
    float kd;

    float set;
    float get;
    float err;

    float max_out;
    float max_iout;

    float Pout;
    float Iout;
    float Dout;

    float out;

} M6020_PID_t;


typedef struct
{
    const motor_measure_t *gimbal_motor_measure;  //电机数据结构体
    M6020_PID_t gimbal_motor_relative_angle_pid;
    pid_type_def gimbal_motor_gyro_pid;
    uint16_t offset_ecd;
    float max_relative_angle; //rad
    float min_relative_angle; //rad

    float relative_angle;     //rad
    float relative_angle_set; //rad

    float motor_gyro;         //rad/s
    float motor_gyro_set;
    float motor_speed;
    float raw_cmd_current;
    float current_set;
    int16_t given_current;

} motor_6020_t;



typedef struct
{
    //const RC_ctrl_t *gimbal_rc_ctrl;
    const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
    motor_6020_t M6020_M1;
    motor_6020_t M6020_M2;
    motor_6020_t M6020_M3;
    motor_6020_t M6020_M4;

} M6020_control_t;

static void motor_init(M6020_control_t *init);
static float M6020_PID_calc(M6020_PID_t *pid, float get, float set, float error_delta);
static void M6020_PID_init(M6020_PID_t *pid, float maxout, float max_iout, float kp, float ki, float kd);
static void M6020_control_loop(M6020_control_t *control_loop);
static void M6020_motor_relative_angle_control(motor_6020_t *gimbal_motor);
static void gimbal_feedback_update(M6020_control_t *feedback_update);
static void M6020_PID_clear(M6020_PID_t *M6020_pid_clear);

#endif 
