#include "status.h"


char status_change_callback(RC_ctrl_t *_rc)
{
	if(_rc->rc.s[0]==2&&_rc->rc.s[1]==2)
		return Reset_system;
	else if(_rc->rc.s[0]==2&&_rc->rc.s[1]==1)
		return 0;
	else if(_rc->rc.s[0]==2&&_rc->rc.s[1]==3)
		return 0;
	else if(_rc->rc.s[0]==1&&_rc->rc.s[1]==1)
		return Status_shoot_low;
	else if(_rc->rc.s[0]==1&&_rc->rc.s[1]==3)
		return Status_shoot_mid;
	else if(_rc->rc.s[0]==1&&_rc->rc.s[1]==2)
		return Status_shoot_high;
	else if(_rc->rc.s[0]==3&&_rc->rc.s[1]==1)
		return Status_chassis_and_pick;
	else if(_rc->rc.s[0]==3&&_rc->rc.s[1]==3)
		return Status_chassis_and_pick;
	else if(_rc->rc.s[0]==3&&_rc->rc.s[1]==2)
		return Status_chassis_and_pick;
	
	else
		return 0;
}
char status_mode_change_callback(RC_ctrl_t *_rc)
{
	if(_rc->rc.ch[4]>=500)
		return mode_1;
	else if(_rc->rc.ch[4]<=-500)
		return mode_2;
	else 
		return mode_0;
}

REMOTE status_remote_callback(RC_ctrl_t *_rc)
{
	REMOTE _remote;
	_remote.ch[0] = _rc->rc.ch[0];
	_remote.ch[1] = _rc->rc.ch[1];
	_remote.ch[2] = _rc->rc.ch[2];
	_remote.ch[3] = _rc->rc.ch[3];
	_remote.ch[4] = _rc->rc.ch[4]; 
	
	_remote.s[0] = _rc->rc.s[0];
	_remote.s[1] = _rc->rc.s[1];
	
	return _remote;
}





