#include "main.h"
#include "cmsis_os.h"
#include "pid.h"
#include "chassis.h"
#include "can_recv.h"
#include "user_lib.h"

/*对应电机数据,0~3 为1~4号动力电机3508,4~7为1~4号航向电机6020

   |                 ^ Y方向
   | 一号(6020)      |        二号(6020)                      
   |     (3508)      |           (3508)             
   |                 |                                   
   |   ——————————————|————————————————> X方向
   |                 |                                  
   | 三号(6020)      |       四号（6020）               
   |     (3508)      |          (3508)      
   |                 |

*/


extern motor_measure_t    motor_Date[7]; 
M6020_control_t motor_control;

static int16_t M1_can_set_current = 0;

#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#define gimbal_total_pid_clear(gimbal_clear)                                            \
    {                                                                                   \
        M6020_PID_clear(&(gimbal_clear)->M6020_M1.gimbal_motor_relative_angle_pid);    \
        PID_clear(&(gimbal_clear)->M6020_M1.gimbal_motor_gyro_pid);                     \
                                                                                        \
        M6020_PID_clear(&(gimbal_clear)->M6020_M1.gimbal_motor_relative_angle_pid);    \
        PID_clear(&(gimbal_clear)->M6020_M1.gimbal_motor_gyro_pid);                     \
    }                                                                                   \

static void motor_init(M6020_control_t *init);
static fp32 M6020_PID_calc(M6020_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);
static void M6020_PID_init(M6020_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
static void M6020_control_loop(M6020_control_t *control_loop);
static void M6020_motor_relative_angle_control(motor_6020_t *gimbal_motor);
static void gimbal_feedback_update(M6020_control_t *feedback_update);
static void M6020_PID_clear(M6020_PID_t *M6020_pid_clear);
//主线程
void chassisTask02(void const * argument)
{
    motor_init(&motor_control);
    while (1)
    {

        gimbal_feedback_update(&motor_control);
        M6020_control_loop(&motor_control);

    }
      
}

static void motor_init(M6020_control_t *init)
{

    static const fp32 M6020_M1_speed_pid[3] = {M6020_MOTOR_SPEED_PID_KP, M6020_MOTOR_SPEED_PID_KI, M6020_MOTOR_SPEED_PID_KD};
    //static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    //电机数据指针获取
    init->M6020_M1.gimbal_motor_measure = get_6020_M1_motor_measure_point();
    init->M6020_M2.gimbal_motor_measure = get_6020_M2_motor_measure_point();
    init->M6020_M3.gimbal_motor_measure = get_6020_M3_motor_measure_point();
    init->M6020_M4.gimbal_motor_measure = get_6020_M4_motor_measure_point();

    //遥控器数据指针获取
    //init->gimbal_rc_ctrl = get_remote_control_point();
    //初始化电机模式
    //init->M6020_M1.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    //init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    //初始化yaw电机pid
    M6020_PID_init(&init->M6020_M1.gimbal_motor_relative_angle_pid, M6020_MOTOR_POSION_PID_MAX_OUT, M6020_MOTOR_POSION_PID_MAX_IOUT, M6020_MOTOR_POSION_PID_KP, M6020_MOTOR_POSION_PID_KI, M6020_MOTOR_POSION_PID_KD);
    //M6020_PID_init(&init->M6020_M2.gimbal_motor_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
    PID_init(&init->M6020_M1.gimbal_motor_gyro_pid, PID_POSITION, M6020_M1_speed_pid, M6020_MOTOR_SPEED_PID_MAX_OUT, M6020_MOTOR_SPEED_PID_MAX_IOUT);
    //初始化pitch电机pid
    //清除所有PID
    gimbal_total_pid_clear(init);
    gimbal_feedback_update(&motor_control);
    init->M6020_M1.relative_angle_set = init->M6020_M1.relative_angle;
    init->M6020_M1.motor_gyro_set = init->M6020_M1.motor_gyro;

}


static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > 4096)
    {
        relative_ecd -= 8191;
    }
    else if (relative_ecd < -4096)
    {
        relative_ecd += 8191;
    }
    return relative_ecd * MOTOR_ECD_TO_RAD;
}



static void M6020_PID_init(M6020_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}


static fp32 M6020_PID_calc(M6020_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

static void M6020_PID_clear(M6020_PID_t *M6020_pid_clear)
{
    if (M6020_pid_clear == NULL)
    {
        return;
    }
    M6020_pid_clear->err = M6020_pid_clear->set = M6020_pid_clear->get = 0.0f;
    M6020_pid_clear->out = M6020_pid_clear->Pout = M6020_pid_clear->Iout = M6020_pid_clear->Dout = 0.0f;
}


static void M6020_control_loop(M6020_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }

        M6020_motor_relative_angle_control(&control_loop->M6020_M1);

}

static void M6020_motor_relative_angle_control(motor_6020_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    //角度环，速度环串级pid调试
    gimbal_motor->motor_gyro_set = M6020_PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    //控制值赋值
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

static void gimbal_feedback_update(M6020_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }
    //云台数据更新
    feedback_update->M6020_M1.relative_angle = motor_ecd_to_angle_change(feedback_update->M6020_M1.gimbal_motor_measure->ecd,
                                                                                          feedback_update->M6020_M1.offset_ecd);

}


