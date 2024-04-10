#include "main.h"
#include "can_recv.h"

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

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static motor_measure_t      motor_Date[7];           //电机回传数据结构体
static CAN_TxHeaderTypeDef  RM6020_tx_message;          //can_6020发送邮箱
static CAN_TxHeaderTypeDef  RM3508_tx_message;         //can_3508发送邮箱

static uint8_t              can_6020_send_data[8];
static uint8_t              can_3508_send_data[8]; 


#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

//所有6020电机（航向电机）的指令发送
void CAN_cmd_6020(int16_t CMD_ID_1, int16_t CMD_ID_2, int16_t CMD_ID_3, int16_t CMD_ID_4)
{
    uint32_t send_mail_box;
    RM6020_tx_message.StdId = CAN_6020_ALL_ID;
    RM6020_tx_message.IDE = CAN_ID_STD;
    RM6020_tx_message.RTR = CAN_RTR_DATA;
    RM6020_tx_message.DLC = 0x08;
    can_6020_send_data[0] = (CMD_ID_1 >> 8);
    can_6020_send_data[1] = CMD_ID_1;
    can_6020_send_data[2] = (CMD_ID_2 >> 8);
    can_6020_send_data[3] = CMD_ID_2;
    can_6020_send_data[4] = (CMD_ID_3 >> 8);
    can_6020_send_data[5] = CMD_ID_3;
    can_6020_send_data[6] = (CMD_ID_4 >> 8);
    can_6020_send_data[7] = CMD_ID_4;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &RM6020_tx_message, can_6020_send_data, &send_mail_box);
}

//所有3508电机（动力电机）的指令发送
void CAN_cmd_3508(int16_t CMD_ID_1, int16_t CMD_ID_2, int16_t CMD_ID_3, int16_t CMD_ID_4)
{
    uint32_t send_mail_box;
    RM3508_tx_message.StdId = CAN_3508_ALL_ID;
    RM3508_tx_message.IDE = CAN_ID_STD;
    RM3508_tx_message.RTR = CAN_RTR_DATA;
    RM3508_tx_message.DLC = 0x08;
    can_3508_send_data[0] = (CMD_ID_1 >> 8);
    can_3508_send_data[1] = CMD_ID_1;
    can_3508_send_data[2] = (CMD_ID_2 >> 8);
    can_3508_send_data[3] = CMD_ID_2;
    can_3508_send_data[4] = (CMD_ID_3 >> 8);
    can_3508_send_data[5] = CMD_ID_3;
    can_3508_send_data[6] = (CMD_ID_4 >> 8);
    can_3508_send_data[7] = CMD_ID_4;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &RM3508_tx_message, can_3508_send_data, &send_mail_box);
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:

        case CAN_6020_M1_ID:
        case CAN_6020_M2_ID:
        case CAN_6020_M3_ID:
        case CAN_6020_M4_ID:
        {
            static uint8_t i = 0;
            i = rx_header.StdId - CAN_3508_M1_ID;
            get_motor_measure(&motor_Date[i], rx_data);
          
            break;
        }

        default:
        {
            break;
        }
    }
}


const motor_measure_t *get_3508_M1_motor_measure_point(void)
{
    return &motor_Date[0];
}
const motor_measure_t *get_3508_M2_motor_measure_point(void)
{
    return &motor_Date[1];
}
const motor_measure_t *get_3508_M3_motor_measure_point(void)
{
    return &motor_Date[2];
}
const motor_measure_t *get_3508_M4_motor_measure_point(void)
{
    return &motor_Date[3];
}
const motor_measure_t *get_6020_M1_motor_measure_point(void)
{
    return &motor_Date[4];
}
const motor_measure_t *get_6020_M2_motor_measure_point(void)
{
    return &motor_Date[5];
}
const motor_measure_t *get_6020_M3_motor_measure_point(void)
{
    return &motor_Date[6];
}
const motor_measure_t *get_6020_M4_motor_measure_point(void)
{
    return &motor_Date[7];
}
