#include "rabbit.h"
#include "status.h"
#include "pid.h"
#include "delay.h"
#include "math.h"

extern CHASSIS _chassis;
extern int Flag_right;
float cos45;

#define FLAG_TURN     -1
#define speed_enlarge 8
	
PidType _PID_Param;


float teat_angle;
/*两个方案

	将目标速度设为 0 
	将电流值设为   0 

*/
void To_Zero(CHASSIS *_chassis)
{

	u8 i;
	for(i=0;i<4;i++){
	PID_calc(&_chassis->motor_3508[i].pid_param_speed,_chassis->motor_3508[i].feedback.speed_rpm,0);

	}

}
	
void system_init(void)
{
	u8 i;
	cos45 = arm_cos_f32(3.14159/4);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);	//设置系统中断优先级分组4	
  for(i=0;i<4;i++)
		PID_init(&_chassis.motor_3508[i].pid_param_speed,PID_POSITION,10.0f,1.0f,0.0f,15000.0f,800.0f,500.0f);
	for(i=0;i<4;i++)
		PID_init(&_chassis.motor_2006[i].pid_param_speed,PID_POSITION,9.0f,0.0f,5.0f,9000.0f,800.0f,5.0f);
	for(i=0;i<4;i++)
		PID_init(&_chassis.motor_2006[i].pid_param_angle,PID_POSITION,0.4f,0.0f,1.5f,8192.0f*36.0f*4.0f,800.0f,5.0f);
	for(i=0;i<4;i++)
		PID_init(&_chassis.MT_angle.pid_param_speed,PID_POSITION,9.0f,0.0f,5.0f,9000.0f,800.0f,5.0f);
	for(i=0;i<4;i++)
		PID_init(&_chassis.MT_angle.pid_param_angle,PID_POSITION,0.0042f,0.0f,0.003f,8192.0f*36.0f*4.0f,800.0f,5.0f);
	delay_init(168);								//初始化延时函数
	gpio_Init();
	SPI1_Init();
	remote_control_init();           //遥控器
	usart1_init(115200);
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
	CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);
	for(i=0;i<4;i++)
		_chassis.motor_2006[i].start_all_ecd=0;
}


void ros_data_can1_send(uint16_t stid , RC_ctrl_t *rc_ctrl ,CHASSIS *_chassis)
{
	CanTxMsg TxMessage;
	TxMessage.StdId = stid;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 0x08;
	
	TxMessage.Data[0] = (_chassis->w_v /2 ) >> 8;
	TxMessage.Data[1] = (_chassis->w_v / 2);
	TxMessage.Data[2] = _chassis->w_v >> 8;
	TxMessage.Data[3] = _chassis->w_v;
	TxMessage.Data[4] = _chassis->l_v >> 8;
	TxMessage.Data[5] = _chassis->l_v;
	TxMessage.Data[6] = rc_ctrl->rc.s[0]<<4 | rc_ctrl->rc.s[1];
	TxMessage.Data[7] = 0;
	
	CAN_Transmit(CAN1,&TxMessage);
}


void ros_data_can2_send(uint16_t stid , RC_ctrl_t *rc_ctrl ,CHASSIS *_chassis)
{
	CanTxMsg TxMessage;
	TxMessage.StdId = stid;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = (_chassis->w_v /5 ) >> 8;
	TxMessage.Data[1] = (_chassis->w_v / 5);
	TxMessage.Data[2] = _chassis->w_v >> 8;
	TxMessage.Data[3] = _chassis->w_v;
	TxMessage.Data[4] = _chassis->l_v >> 8;
	TxMessage.Data[5] = _chassis->l_v;
	TxMessage.Data[6] = rc_ctrl->rc.s[0]<<4 | rc_ctrl->rc.s[1];
	TxMessage.Data[7] = 0;
	
	CAN_Transmit(CAN2,&TxMessage);
}
