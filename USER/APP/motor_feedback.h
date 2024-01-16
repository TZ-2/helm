#ifndef _MOTOR_FEEDBACK_H
#define _MOTOR_FEEDBACK_H

#include "pid.h"
#include "sys.h"


/*              can 1                 */
#define chassis_3508_motor1 0x201
#define chassis_3508_motor2 0x202
#define chassis_3508_motor3 0x203
#define chassis_3508_motor4 0x204
#define mt6816_angle_data   0x210

/*              can 2                 */
#define chassis_2006_motor1 0x201
#define chassis_2006_motor2 0x202
#define chassis_2006_motor3 0x203
#define chassis_2006_motor4 0x204

#define _3508_stdid         0x200
#define _2006_stdid         0x200
/**************************************/


#define get_motor_measure(ptr, rx_message)                                              \
{                                                                                       \
    if((ptr)->ecd - (ptr)->last_ecd > 4096) (ptr)->count-- ;                            \
		else if((ptr)->ecd - (ptr)->last_ecd < -4096 ) (ptr)->count ++ ;											\
    (ptr)->last_ecd = (ptr)->ecd;                                                       \
    (ptr)->ecd = (uint16_t)((rx_message).Data[0] << 8 | (rx_message).Data[1]);          \
    (ptr)->speed_rpm = (uint16_t)((rx_message).Data[2] << 8 |(rx_message).Data[3]);     \
    (ptr)->given_current = (uint16_t)((rx_message).Data[4] << 8 | (rx_message).Data[5]); \
    (ptr)->temperate = (rx_message).Data[6];                                             \
    (ptr)->all_ecd=(ptr)->count*8191+(ptr)->ecd;                                          \
}


typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;  //当前 -- current
    int32_t  all_ecd;
    int32_t  count;
    uint8_t temperate;

    int16_t last_ecd;
	float pid_set_speed;
} motor_measure_t;


typedef struct
{
	float target_rpm;      //目标
	float target_angle;   //目标角度
	int   ref_angle[4];
	int   set_mangle;
	int32_t start_all_ecd;
	int32_t right_all_ecd;
	
	int initial_ecd;
	int tern;
	PidType pid_param_speed;
	PidType pid_param_angle;
	
	motor_measure_t feedback; //反馈
}MOTOR;

#include "remote_control.h"

void motor_driver_can1(uint16_t stid, int i1,int i2,int i3,int i4);
void motor_driver_can2(uint16_t stid, int i1,int i2,int i3,int i4);
void rc_data_can1_send(uint16_t stid , RC_ctrl_t *rc_ctrl);
void rc_data_can2_send(uint16_t stid , RC_ctrl_t *rc_ctrl);
void Get_angle(MOTOR *ptr,CanRxMsg rx_message);
void CAN1_RX0_IRQnHandler(void);
void CAN2_RX1_IRQnHandler(void);

void motor_close_callback(MOTOR *_motor);
void motor_speed_pid_cal_callback(MOTOR *_motor, float _target);
void motor_angle_pid_cal_callback(MOTOR *_motor, float _target);
void pid_angle_init_callback(MOTOR *_motor, float _arr[6]);
void pid_speed_init_callback(MOTOR *_motor, float _arr[6]);

#endif
