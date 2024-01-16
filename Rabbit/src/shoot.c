#include "rabbit.h"
#include "status.h"


static float shoot_speed_pid_param[6]={30.0f, 1.0f, 0.0f, 20000.0f, 500.0f, 500.0f};
static float shoot_angle_pid_param[6]={0.3f, 1.0f, 0.0f, 2000.0f, 500.0f, 500.0f};
static char flag1=0;
static char flag2=1;
static char flag3=1;
static char shoot_angle_update=ON;
char shoot_lifting_start=OFF;
extern char pick_up_state;
char shoot_fly_motor_init=OFF;


void shoot_task(void *pvParameters)
{
	LED2=0;
	shoot_init(&rabbit.shoot, 294876);
	while(1)
	{		
		if(status_change_callback(&rc_ctrl)!=Reset_system)
		{
			shoot_push(&rabbit.shoot, status_mode_change_callback, status_remote_callback, 3000, pick_up_motor_speed_set);
			shoot_motor_pid_cal(&rabbit.shoot, motor_speed_pid_cal_callback, motor_angle_pid_cal_callback);
			shoot_motor_drive(&rabbit.shoot, &rabbit.pick_up, 0x200, 0x200, motor_driver_can1, motor_driver_can2);
			
			shoot_lifting(&rabbit.shoot, &rabbit.pick_up, status_mode_change_callback, status_remote_callback);
			
			shoot_fly_motor_speed_set(&rabbit.shoot, status_mode_change_callback, status_change_callback, status_remote_callback);
			shoot_fly_motor_drive(rabbit.shoot.shoot_rpm, status_remote_callback);
				
		}
		else 
		{
			shoot_fly_motor_drive(shoot_rpm_init, status_remote_callback);
		}
		vTaskDelay(1);
	}
}


void shoot_init(SHOOT *_shoot, int _target_angle)
{
	_shoot->push=shoot_Push;
	_shoot->lifting=shoot_Lifting;
	shoot_motor_pid_init(_shoot, pid_speed_init_callback, pid_angle_init_callback);
	
}




/**
	*@brief 
	*@param 
	*@param mode
	*@param status
	*@param remote
	*@retval 
**/
void shoot_fly_motor_speed_set(SHOOT *_shoot, char (*callback1)(RC_ctrl_t *_rc), char (*callback2)(RC_ctrl_t *_rc), REMOTE (*callback3)(RC_ctrl_t *_rc))
{
	static char last_status;
	if(last_status!=(*callback2)(&rc_ctrl) && (*callback1)(&rc_ctrl)==mode_2)
	{
		if((*callback2)(&rc_ctrl)==Status_shoot_high)
			_shoot->shoot_rpm = shoot_rpm_set_high;
		
		else if((*callback2)(&rc_ctrl)==Status_shoot_mid)
			_shoot->shoot_rpm = shoot_rpm_set_mid;
		
		else if((*callback2)(&rc_ctrl)==Status_shoot_low)
			_shoot->shoot_rpm = shoot_rpm_set_low;		
	}
	last_status = (*callback2)(&rc_ctrl);
	
	if((*callback1)(&rc_ctrl)==mode_2&& (*callback3)(&rc_ctrl).s[0]==1)
	{
		if((*callback3)(&rc_ctrl).ch[1]>0 && flag1==1)
		{
			_shoot->shoot_rpm+=1;
			flag1=0;
		}
		else if((*callback3)(&rc_ctrl).ch[1]<0 && flag1==1)
		{
			_shoot->shoot_rpm-=1;
			flag1=0;
		}
		else if((*callback3)(&rc_ctrl).ch[1]==0 && flag1==0)
		{
			flag1=1;
		}
	}
		
	
}



/**
	*@brief 
	*@param 
	*@param mode
	*@param remote
	*@param motor_speed
	*@retval 
**/
void shoot_push(SHOOT *_shoot, char (*callback1)(RC_ctrl_t *_rc), REMOTE (*callback2)(RC_ctrl_t *_rc), int _motor_speed, char (*state)(PICK_UP *_pick_up, SHOOT *_shoot, char (*ch)(RC_ctrl_t *_rc), REMOTE (*remote)(RC_ctrl_t *_rc))
)
{
	if((*callback2)(&rc_ctrl).s[0]==1)
	{
		if((*callback1)(&rc_ctrl)==mode_1 && (*callback2)(&rc_ctrl).ch[1]==0 && flag3)
		{
			_shoot->push(ON);
			flag3=0;
		}
		else if((*callback1)(&rc_ctrl)==mode_0 && (*callback2)(&rc_ctrl).ch[1]==0 && flag3!=1)
		{
			_shoot->push(OFF);
			_shoot->motor.target_rpm = _motor_speed;
			shoot_enter_motor_angle_update(_shoot, 294876);
			flag3=1;
		}
			
	}
	
	if(_shoot->motor.feedback.all_ecd >= _shoot->motor.target_angle)
	{
		_shoot->motor.target_rpm=0;
		_shoot->motor.feedback.all_ecd=0;
		_shoot->motor.feedback.count=0;
		_shoot->motor.target_angle=0;
		shoot_angle_update=ON;
		if((*state)(&rabbit.pick_up, &rabbit.shoot, status_mode_change_callback, status_remote_callback)==6||(*state)(&rabbit.pick_up, &rabbit.shoot, status_mode_change_callback, status_remote_callback)==3)
		{
			pick_up_state++;
		}		
	}
}



/**
	*@brief 
	*@param 
	*@param pid_speed_init_callback
	*@param pid_angle_init_callback
	*@retval 
**/
void shoot_motor_pid_init(SHOOT *_shoot, void (*callback1)(MOTOR *_motor, float _arr[6]), void (*callback2)(MOTOR *_motor, float _arr[6]))
{
	(*callback1)(&_shoot->motor, shoot_speed_pid_param);
	(*callback2)(&_shoot->motor, shoot_angle_pid_param);
}


/**
	*@brief 
	*@param 
	*@param motor_speed_pid_cal_callback
	*@param motor_angle_pid_cal_callback
	*@retval 
**/
void shoot_motor_pid_cal(SHOOT *_shoot, void (*callback1)(MOTOR *_motor, float _target), void (*callback2)(MOTOR *_motor, float _target))
{
	(*callback1)(&_shoot->motor, _shoot->motor.target_rpm);
}




/**
	*@brief 
	*@param 
	*@param	id1
	*@param	id2
	*@param	can1
	*@param	can2
	*@retval 
**/
void shoot_motor_drive(SHOOT *_shoot, PICK_UP *_pick_up,  uint16_t std_id1, uint16_t std_id2, void (*callback1)(uint16_t stid, int i1,int i2,int i3,int i4), void (*callback2)(uint16_t stid, int i1,int i2,int i3,int i4))
{	
	/*worning!! 可能会造成某些bug*/
	(*callback2)(std_id2, 		_shoot->motor.pid_param_speed.out,
								0,
								0,
								0									 );
}


/**
	*@brief 发射驱动 
	*@param 发射速度
	*@retval none
**/
void shoot_fly_motor_drive(int _speed, REMOTE (*remote)(RC_ctrl_t *_rc)
)
{
	if((*remote)(&rc_ctrl).s[0]==1)
	{
		TIM_SetCompare1(TIM5, _speed);
		TIM_SetCompare2(TIM5, _speed);
	}
	else 
	{
		TIM_SetCompare1(TIM5, shoot_rpm_init);
		TIM_SetCompare2(TIM5, shoot_rpm_init);
		
	}
}


/**
	*@brief 
	*@param 
	*@param	mode
	*@param	remote
	*@retval 
**/
char shoot_lifting_set_callback(char (*callback1)(RC_ctrl_t *_rc), REMOTE (*callback2)(RC_ctrl_t *_rc))
{
		if((*callback1)(&rc_ctrl)==mode_1 && (*callback2)(&rc_ctrl).ch[1]>=600 && flag2)
		{
			flag2=0;
			return ON;
		}
		else if((*callback1)(&rc_ctrl)==mode_1 && (*callback2)(&rc_ctrl).ch[1]<=-600 && flag2!=1)
		{
			flag2=1;
			return OFF;		
		}
		else return 2;
			
}


void shoot_lifting(SHOOT *_shoot, PICK_UP *_pick_up, char (*callback1)(RC_ctrl_t *_rc), REMOTE (*callback2)(RC_ctrl_t *_rc))
{
		if((*callback1)(&rc_ctrl)==mode_1 && (*callback2)(&rc_ctrl).ch[1]>=600 && flag2 && (*callback2)(&rc_ctrl).s[0]==1)
		{
			flag2=0;
			_pick_up->motor[0].feedback.all_ecd=0;			
			_pick_up->motor[0].feedback.count=0;
			_pick_up->motor[0].target_rpm=0;
		
			_pick_up->motor[0].target_rpm=-500;
			_pick_up->motor[1].target_rpm=-_pick_up->motor[0].target_rpm;
			
			_pick_up->motor[0].target_angle=-45876;
			shoot_lifting_start=ON;

		}
		else if((*callback1)(&rc_ctrl)==mode_1 && (*callback2)(&rc_ctrl).ch[1]<=-600 && flag2!=1 && (*callback2)(&rc_ctrl).s[0]==1)
		{
			flag2=1;
			_pick_up->motor[0].feedback.count=0;
			_pick_up->motor[0].feedback.all_ecd=0;
			_pick_up->motor[0].target_rpm=0;
			
			_shoot->lifting(OFF);
			
			_pick_up->motor[0].target_rpm=700;
			_pick_up->motor[1].target_rpm=-_pick_up->motor[0].target_rpm;
			
			_pick_up->motor[0].target_angle=59876;
		
		}		
}


void shoot_Push(int x)
{
	Push=x;
}

void shoot_Lifting(int x)
{
	Lifting=x;
}

void shoot_enter_motor_angle_update(SHOOT *_shoot, int _angle)
{
	if(shoot_angle_update)
	{
		_shoot->motor.target_angle=_angle;
		shoot_angle_update=OFF;
	}
	
}

