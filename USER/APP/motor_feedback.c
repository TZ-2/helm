#include "motor_feedback.h"
#include "rabbit.h"
#include "can.h"


extern CHASSIS _chassis;
extern int Flag_init;

void motor_speed_pid_cal_callback(MOTOR *_motor, float _target)
{
    PID_calc(&_motor->pid_param_speed, _motor->feedback.speed_rpm, _target);
}


void motor_angle_pid_cal_callback(MOTOR *_motor, float _target)
{
    PID_calc(&_motor->pid_param_angle, _motor->feedback.all_ecd, _target);
	PID_calc(&_motor->pid_param_speed, _motor->feedback.speed_rpm, _motor->pid_param_angle.out);
}


void motor_close_callback(MOTOR *_motor)
{
	PID_calc(&_motor->pid_param_speed, _motor->feedback.speed_rpm, 0);
}


void pid_speed_init_callback(MOTOR *_motor, float _arr[6])
{
	PID_init(&_motor->pid_param_speed, PID_POSITION, _arr[0], _arr[1], _arr[2], _arr[3], _arr[4], _arr[5]);
}

void pid_angle_init_callback(MOTOR *_motor, float _arr[6])
{
	PID_init(&_motor->pid_param_angle, PID_POSITION, _arr[0], _arr[1], _arr[2], _arr[3], _arr[4], _arr[5]);
}



void motor_driver_can1(uint16_t stid, int i1,int i2,int i3,int i4)
{
	CanTxMsg TxMessage;
	
	TxMessage.DLC=8;
	TxMessage.IDE=CAN_Id_Standard;
	TxMessage.RTR=CAN_RTR_Data;
	TxMessage.StdId=stid;
	
	TxMessage.Data[0]=i1 >> 8;
	TxMessage.Data[1]=i1;
	TxMessage.Data[2]=i2 >> 8;
	TxMessage.Data[3]=i2;
	TxMessage.Data[4]=i3 >> 8;
	TxMessage.Data[5]=i3;
	TxMessage.Data[6]=i4 >> 8;
	TxMessage.Data[7]=i4; 
	
	CAN_Transmit(CAN1,&TxMessage);
}


void rc_data_can1_send(uint16_t stid , RC_ctrl_t *rc_ctrl)
{
	CanTxMsg TxMessage;
	TxMessage.StdId = stid;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 0x08;
	
	TxMessage.Data[0] = rc_ctrl->rc.ch[0] >> 8;
	TxMessage.Data[1] = rc_ctrl->rc.ch[0];
	TxMessage.Data[2] = rc_ctrl->rc.ch[2] >> 8;
	TxMessage.Data[3] = rc_ctrl->rc.ch[2];
	TxMessage.Data[4] = rc_ctrl->rc.ch[3] >> 8;
	TxMessage.Data[5] = rc_ctrl->rc.ch[3];
	TxMessage.Data[6] = rc_ctrl->rc.s[0]<<4 | rc_ctrl->rc.s[1];
	TxMessage.Data[7] = 0;
	
	CAN_Transmit(CAN1,&TxMessage);
}

void rc_data_can2_send(uint16_t stid , RC_ctrl_t *rc_ctrl)
{
	CanTxMsg TxMessage;
	TxMessage.StdId = stid;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = 0x08;
	
	TxMessage.Data[0] = rc_ctrl->rc.ch[0] >> 8;     //这里左移完数据不就没了吗
	TxMessage.Data[1] = rc_ctrl->rc.ch[0];
	TxMessage.Data[2] = rc_ctrl->rc.ch[2] >> 8;
	TxMessage.Data[3] = rc_ctrl->rc.ch[2];
	TxMessage.Data[4] = rc_ctrl->rc.ch[3] >> 8;
	TxMessage.Data[5] = rc_ctrl->rc.ch[3];
	TxMessage.Data[6] = rc_ctrl->rc.s[0]<<4 | rc_ctrl->rc.s[1];
	TxMessage.Data[7] = 0;
	
	CAN_Transmit(CAN2,&TxMessage);
}

void motor_driver_can2(uint16_t stid, int i1,int i2,int i3,int i4)
{
	
	CanTxMsg TxMessage;	
	TxMessage.DLC=8;
	TxMessage.IDE=CAN_Id_Standard;
	TxMessage.RTR=CAN_RTR_Data;
	TxMessage.StdId=stid;
	
	TxMessage.Data[0]=i1 >> 8;
	TxMessage.Data[1]=i1;
	TxMessage.Data[2]=i2 >> 8;
	TxMessage.Data[3]=i2;
	TxMessage.Data[4]=i3 >> 8;
	TxMessage.Data[5]=i3;
	TxMessage.Data[6]=i4 >> 8;
	TxMessage.Data[7]=i4; 
	
	CAN_Transmit(CAN2,&TxMessage);

}


void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg Rx1Message;
	CAN_Receive(CAN1,CAN_FIFO0,&Rx1Message);
	switch(Rx1Message.StdId)
	{
		case chassis_3508_motor1: get_motor_measure(&_chassis.motor_3508[0].feedback, Rx1Message);break;
		case chassis_3508_motor2: get_motor_measure(&_chassis.motor_3508[1].feedback, Rx1Message);break;
		case chassis_3508_motor3: get_motor_measure(&_chassis.motor_3508[2].feedback, Rx1Message);break;
		case chassis_3508_motor4: get_motor_measure(&_chassis.motor_3508[3].feedback, Rx1Message);break;		
		case mt6816_angle_data:   Get_angle(&_chassis.MT_angle, Rx1Message);break;
	}

}


void CAN2_RX1_IRQHandler(void)
{
	CanRxMsg Rx2Message;
	CAN_Receive(CAN2,CAN_FIFO1,&Rx2Message);
	switch(Rx2Message.StdId)
	{
		case chassis_2006_motor1: get_motor_measure(&_chassis.motor_2006[0].feedback, Rx2Message);break;
    case chassis_2006_motor2: get_motor_measure(&_chassis.motor_2006[1].feedback, Rx2Message);break;
    case chassis_2006_motor3: get_motor_measure(&_chassis.motor_2006[2].feedback, Rx2Message);break;
    case chassis_2006_motor4: get_motor_measure(&_chassis.motor_2006[3].feedback, Rx2Message);break;
 	}
	if(Flag_init == 0){
		_chassis.motor_2006[0].start_all_ecd=_chassis.motor_2006[0].feedback.all_ecd;
		_chassis.motor_2006[1].start_all_ecd=_chassis.motor_2006[1].feedback.all_ecd;
		_chassis.motor_2006[2].start_all_ecd=_chassis.motor_2006[2].feedback.all_ecd;
		_chassis.motor_2006[3].start_all_ecd=_chassis.motor_2006[3].feedback.all_ecd;
		Flag_init = 1;
	}
}

void Get_angle(MOTOR *ptr,CanRxMsg rx_message)
{
  ptr->ref_angle[0] = rx_message.Data[0];
	ptr->ref_angle[1] = rx_message.Data[3];
	ptr->ref_angle[2] = rx_message.Data[5];
	ptr->ref_angle[3] = rx_message.Data[7];
	
}
//void CAN2_RX1_IRQHandler(void)
//{
//	CanRxMsg Rx2Message;
//	CAN_Receive(CAN2,CAN_FIFO1,&Rx2Message);
//	switch(Rx2Message.StdId)
//	{
//		case chassis_2006_motor1: get_motor_measure(&_chassis.motor_2006[0].feedback, Rx2Message);break;
//    case chassis_2006_motor2: get_motor_measure(&_chassis.motor_2006[1].feedback, Rx2Message);break;
//    case chassis_2006_motor3: get_motor_measure(&_chassis.motor_2006[2].feedback, Rx2Message);break;
//    case chassis_2006_motor4: get_motor_measure(&_chassis.motor_2006[3].feedback, Rx2Message);break;
// 	}
//	if(Flag == 0){
//		_chassis.motor_2006[0].start_all_ecd=_chassis.motor_2006[0].feedback.all_ecd;
//		_chassis.motor_2006[1].start_all_ecd=_chassis.motor_2006[1].feedback.all_ecd;
//		_chassis.motor_2006[2].start_all_ecd=_chassis.motor_2006[2].feedback.all_ecd;
//		_chassis.motor_2006[3].start_all_ecd=_chassis.motor_2006[3].feedback.all_ecd;
//		//Flag=1;
//	}
//}


//void CAN2_RX1_IRQHandler(void)
//{
//	CanRxMsg Rx2Message;
//	CAN_Receive(CAN2,CAN_FIFO1,&Rx2Message);
//	switch(Rx2Message.StdId)
//	{
//		case 0x205: get_motor_measure(&_chassis.motor_3508[0].feedback, Rx2Message);break;
//		case 0x206: get_motor_measure(&_chassis.motor_3508[1].feedback, Rx2Message);break;
//		case chassis_3508_motor3: get_motor_measure(&_chassis.motor_3508[2].feedback, Rx2Message);break;
//		case chassis_3508_motor4: get_motor_measure(&_chassis.motor_3508[3].feedback, Rx2Message);break;		
//	}
//}

