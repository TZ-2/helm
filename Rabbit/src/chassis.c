#include "rabbit.h"
#include "status.h"
#include "Remote_Control.h"
#include "motor_feedback.h"

CHASSIS _chassis;
extern int Flag_right;


//*******************************************底盘运动 | 底盘任务********************************************************//
void CHASSIS_TASK(void *pvParameters)
{
	USART_SendData(USART1,0x11);
	while(1){
	if(rc_ctrl.rc.s[0] == 1 && rc_ctrl.rc.s[1] == 1)
	{
		ros_data_can2_send(0x212,&rc_ctrl,&_chassis);
		ros_data_can2_send(0x211,&rc_ctrl,&_chassis);
		ros_data_can1_send(0x210,&rc_ctrl,&_chassis);
		ros_data_can1_send(0x213,&rc_ctrl,&_chassis);
	}
	else if(rc_ctrl.rc.s[0] == 3 && rc_ctrl.rc.s[1] == 3)
	{
		rc_data_can2_send(0x212,&rc_ctrl);
		rc_data_can2_send(0x211,&rc_ctrl);
		rc_data_can1_send(0x210,&rc_ctrl);
		rc_data_can1_send(0x213,&rc_ctrl);
	}
	vTaskDelay(1);	
	}

}

