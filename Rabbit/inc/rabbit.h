#ifndef _RABBIT_H
#define _RABBIT_H


#include "stdlib.h"
#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "task.h"
#include "FreeRTOSConfig.h"
#include "motor_feedback.h"
#include "Remote_Control.h"
#include "arm_math.h"
#include "gpio.h"
#include "can.h"
#include "usart.h"
#include "spi.h"
#include "timer.h"           
#include "delay.h"
#include "sys.h"
#include "stdio.h"
#include "dead_zone.h"
#include "status.h"
#include "Low_pass.h"
#include "usart.h"
#include "stdio.h"
#include "adc.h"


typedef enum
{
	CHASSIS_NOPOWER=0,
	CHASSIS_GYRO,
	CHASSIS_NORMAL,
}CHASSIS_MODE;



typedef struct
{	
	int max_straight_speed;
	int max_spin_speed;
	
	int chassis_mode;
	int chassis_last_mode;
	
	int l_v;
	int w_v;
	MOTOR motor_3508[4];
	MOTOR motor_2006[4];
	
	MOTOR MT_angle;
	
	int angle_slove[4];
}CHASSIS;




/*****************************   rabbit.c   **********************************/
void CHASSIS_TASK(void *pvParameters);
void system_init(void);
void Get_Right(CHASSIS *_chassis);
/*****************************************************************************/


/**************************************  chassis.c   **************************************/
void To_Zero(CHASSIS *_chassis);
void chassis_behavier(RC_ctrl_t *RC_CTRL , CHASSIS *_chassis);
void chassis_feedback_update(MOTOR *motor_rmp);
void chassis_rc_date_slove(RC_ctrl_t *RC_CTRL,CHASSIS *_chassis);
void chassis_behavier_get(CHASSIS *_chassis);
void rc_data_package(RC_ctrl_t *rc_ctrl);
/******************************************************************************************/

/**************************************  data_send.c   **************************************/
void ros_data_can1_send(uint16_t stid , RC_ctrl_t *rc_ctrl , CHASSIS *_chassis);
void ros_data_can2_send(uint16_t stid , RC_ctrl_t *rc_ctrl , CHASSIS *_chassis) ;
/******************************************************************************************/

#endif



