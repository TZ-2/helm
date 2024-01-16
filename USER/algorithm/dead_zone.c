#include "dead_zone.h"
#include "rabbit.h"

static char dead_zone_param=66;
DEAD_ZONE dead_zone_output(RC_ctrl_t *_rc, REMOTE (*callback)(RC_ctrl_t *_rc))
{
	DEAD_ZONE _remote;
	
	_remote.ch[0]=dead_zone_change((*callback)(&rc_ctrl).ch[0], dead_zone_param);
	_remote.ch[1]=dead_zone_change((*callback)(&rc_ctrl).ch[1], dead_zone_param);
	_remote.ch[2]=dead_zone_change((*callback)(&rc_ctrl).ch[2], dead_zone_param);
	_remote.ch[3]=dead_zone_change((*callback)(&rc_ctrl).ch[3], dead_zone_param);
	_remote.ch[4]=dead_zone_change((*callback)(&rc_ctrl).ch[4], dead_zone_param);
	
	_remote.s[0] = _rc->rc.s[0];
	_remote.s[1] = _rc->rc.s[1];
	
	return _remote;
}

double dead_zone_change(double _input, char _set)
{
	double _output;
	
	if(_input>=0)
	{
		_input-=_set;
		if(_input>=0)
			_output=_input;
		else _output=0;
	}
	else if(_input<0)
	{
		_input+=_set;
		if(_input<=0)
			_output=_input;
		else _output=0;
	}
	_output = _output*660/(660-_set);
	return _output;
	
}



