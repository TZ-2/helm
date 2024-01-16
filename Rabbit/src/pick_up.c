#include "rabbit.h"

static float pick_up_speed_pid_param[6]={20.0f, 1.0f, 1.0f, 10000.0f, 500.0f, 500.0f};
static float pick_up_angle_pid_param[6]={0.3f, 1.0f, 1.0f, 2000.0f, 500.0f, 500.0f};

char pick_up_chassis_auto=OFF;
char pick_up_state=0;
extern char shoot_lifting_start;
static char flag1=1;

void pick_up_task(void *pvParameters)
{
	pick_up_init(&rabbit.pick_up);
	while(1)
	{
		pick_up_motor_speed_set(&rabbit.pick_up, &rabbit.shoot, status_mode_change_callback, status_remote_callback);
		pick_up_motor_pid_cal(&rabbit.pick_up, motor_speed_pid_cal_callback);
		pick_up_motor_drive(&rabbit.pick_up, 0x1ff, motor_driver_can1, &rabbit.shoot);
		
		vTaskDelay(1);
	}
}

void pick_up_init(PICK_UP *_pick_up)
{
	_pick_up->clamping=pick_up_Clamping;
	_pick_up->clamping(OFF);
	pick_up_pid_init(&rabbit.pick_up, pid_speed_init_callback, pid_angle_init_callback);
}


/**
    	*@brief 
    	*@param
    	*@param	low_pass
		*@param	mode
		*@param remote
		*@param	max_speed
    	*@retval 
    **/    	
char pick_up_motor_speed_set(PICK_UP *_pick_up, SHOOT *_shoot, char (*mode)(RC_ctrl_t *_rc), REMOTE (*remote)(RC_ctrl_t *_rc))
{		
/****************************ʰȡ*****************************/
	if((*remote)(&rc_ctrl).s[0]==3 && shoot_lifting_start!=ON && (*remote)(&rc_ctrl).s[1]==1)
	{
		if(pick_up_state==0&&(*mode)(&rc_ctrl)==mode_1)
		{
			_pick_up->motor[0].feedback.count=0;
			_pick_up->motor[0].feedback.all_ecd=0;		
			
			_pick_up->motor[0].target_rpm=-500;
			_pick_up->motor[1].target_rpm=-_pick_up->motor[0].target_rpm;;
			_pick_up->motor[0].target_angle=-50876;
			
			pick_up_state++;
		}
		else if(pick_up_state==1&&(*mode)(&rc_ctrl)==mode_2)
		{
			_pick_up->clamping(ON);
			pick_up_chassis_auto=ON;
			_pick_up->motor[0].feedback.count=0;
			_pick_up->motor[0].feedback.all_ecd=0;		
			
			_pick_up->motor[0].target_rpm=700;
			_pick_up->motor[1].target_rpm=-_pick_up->motor[0].target_rpm;			
			pick_up_state++;
		}
		else if(pick_up_state==2&&(*mode)(&rc_ctrl)==mode_1)
		{
			_pick_up->motor[0].target_rpm=0;
			_pick_up->motor[1].target_rpm=0;
			
			_shoot->motor.target_rpm=3000;
			shoot_enter_motor_angle_update(_shoot, 147438);
			pick_up_state++;
		}
		
		else if(pick_up_state==5)
		{
			_pick_up->clamping(OFF);
			_shoot->motor.target_rpm=3000;
			shoot_enter_motor_angle_update(_shoot, 737190);
			pick_up_state++;
		}
		else if(pick_up_state==7) pick_up_state=0;
		
		flag1=1;
	}
		
		
/****************************ʰȡ�ָ�*****************************/
	else if((*remote)(&rc_ctrl).s[0]==3 && shoot_lifting_start!=ON && (*remote)(&rc_ctrl).s[1]==2)
	{
		if((*mode)(&rc_ctrl)==mode_1&&flag1)
		{
			_pick_up->motor[0].feedback.count=0;
			_pick_up->motor[0].feedback.all_ecd=0;		
			_pick_up->motor[0].target_rpm=-100;
			_pick_up->motor[1].target_rpm=100;
			_pick_up->motor[0].target_angle=-20876;
			flag1=0;
		}
		else if((*mode)(&rc_ctrl)==mode_2)
		{
			_pick_up->clamping(OFF);
			pick_up_state=1;
		}			
		
			
	}
/****************************************************************/
	if((_pick_up->motor[0].target_angle<=_pick_up->motor[0].feedback.all_ecd) && (_pick_up->motor[0].target_angle>0))
	{
		_pick_up->motor[0].feedback.count=0;
		_pick_up->motor[0].feedback.all_ecd=0;
		_pick_up->motor[0].target_rpm=0;
		_pick_up->motor[0].target_angle=0;
		_pick_up->motor[1].feedback.count=0;
		_pick_up->motor[1].feedback.all_ecd=0;
		_pick_up->motor[1].target_rpm=0;
		_pick_up->motor[1].target_angle=0;
		
		if(shoot_lifting_start)
		{
			_shoot->lifting(OFF);
			shoot_lifting_start=OFF;
		}		
	}
	else if(_pick_up->motor[0].target_angle>=_pick_up->motor[0].feedback.all_ecd && _pick_up->motor[0].target_angle<0)
	{
		_pick_up->motor[0].feedback.count=0;
		_pick_up->motor[0].feedback.all_ecd=0;
		_pick_up->motor[0].target_rpm=0;
		_pick_up->motor[0].target_angle=0;		
		_pick_up->motor[1].feedback.count=0;
		_pick_up->motor[1].feedback.all_ecd=0;
		_pick_up->motor[1].target_rpm=0;
		_pick_up->motor[1].target_angle=0;
		if(shoot_lifting_start)
			_shoot->lifting(ON);
	}
	
	return pick_up_state;

	
}



/**
	*@brief 
	*@param mode
	*@param	remote
	*@retval 
**/
void pick_up_clamping(PICK_UP *_pick_up, char (*callback1)(RC_ctrl_t *_rc), REMOTE (*callback2)(RC_ctrl_t *_rc))
{
	if((*callback1)(&rc_ctrl)==mode_0 && (*callback2)(&rc_ctrl).s[0]==3)
	{
		if((*callback2)(&rc_ctrl).s[1]==3)
			_pick_up->clamping(OFF);
		else if((*callback2)(&rc_ctrl).s[1]==1)
			_pick_up->clamping(ON);
	}
}
	


/**
	*@brief 
	*@param 
	*@param pid_speed_init_callback
	*@param pid_angle_init_callback
	*@retval 
**/
void pick_up_pid_init(PICK_UP *_pick_up, void (*callback1)(MOTOR *_motor, float _arr[6]), void (*callback2)(MOTOR *_motor, float _arr[6]))
{
	(*callback1)(&_pick_up->motor[0], pick_up_speed_pid_param);//speed callback1
	(*callback2)(&_pick_up->motor[0], pick_up_angle_pid_param);//angle callback2
	(*callback1)(&_pick_up->motor[1], pick_up_speed_pid_param);//speed callback1
	(*callback2)(&_pick_up->motor[1], pick_up_angle_pid_param);//angle callback2
}
	
void pick_up_motor_pid_cal(PICK_UP *_pick_up, void (*callback)(MOTOR *_motor, float _target))
{
	(*callback)(&_pick_up->motor[0], _pick_up->motor[0].target_rpm);
	(*callback)(&_pick_up->motor[1], _pick_up->motor[1].target_rpm);
}

void pick_up_motor_drive(PICK_UP *_pick_up, uint16_t std_id, void (*callback)(uint16_t stid, int i1,int i2,int i3,int i4), SHOOT *_shoot)
{
	(*callback)(std_id, _pick_up->motor[0].pid_param_speed.out, 
						_pick_up->motor[1].pid_param_speed.out, 
						0, 
						0 );
}

void pick_up_Clamping(int x)
{
	Clamping=x;
}


