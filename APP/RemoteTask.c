#include <stm32f4xx.h>
#include "main.h"


FrictionWheelState_e FrictionWheelState;
Shoot_State_e ShootState;
RampGen_t frictionRamp = RAMP_GEN_DAFAULT;		//摩擦轮斜坡
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   	//键盘速度斜坡
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;
uint16_t remoteShootDelay = 500;
static uint32_t RotateCNT = 0;	//长按连发计数
static uint16_t CNT_1s = 75;		//用于避免四连发模式下两秒内连射8发过于密集的情况
static uint16_t CNT_250ms = 18;	

//遥控器开启摩擦轮
void RemoteShootControl(RemoteSwitch_t *sw, uint8_t val) 
{
	switch(FrictionWheelState)
	{
		case FRICTION_WHEEL_OFF:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_1TO3)   
			{
				ShootState = NOSHOOTING;
				frictionRamp.ResetCounter(&frictionRamp);
				FrictionWheelState = FRICTION_WHEEL_START_TURNNING;	 
			}				 		
		}break;
		case FRICTION_WHEEL_START_TURNNING:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   
			{
				ShootState = NOSHOOTING;
				SetFrictionWheelSpeed(1000);
				FrictionWheelState = FRICTION_WHEEL_OFF;
				frictionRamp.ResetCounter(&frictionRamp);
			}
			else
			{
				SetFrictionWheelSpeed(1000 + (FRICTION_WHEEL_MAX_DUTY-1000)*frictionRamp.Calc(&frictionRamp)); 
				if(frictionRamp.IsOverflow(&frictionRamp))
				{
					FrictionWheelState = FRICTION_WHEEL_ON; 	
				}
				
			}
		}break;
		case FRICTION_WHEEL_ON:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   
			{
				FrictionWheelState = FRICTION_WHEEL_OFF;				  
				SetFrictionWheelSpeed(1000); 
				frictionRamp.ResetCounter(&frictionRamp);
				ShootState = NOSHOOTING;
			}
			else if(sw->switch_value_raw == 2)
			{
				ShootState = SHOOTING;
			}
			else
			{
				ShootState = NOSHOOTING;
			}					 
		} break;				
	}
}

uint8_t rc_data[18];
RC_Ctl_t RC_CtrlData;
InputMode_e inputmode = REMOTE_INPUT; 
ChassisSpeed_Ref_t ChassisSpeedRef; 
float bullet_ref = 0;
float bullet2_ref = 0;
extern FrictionWheelState_e FrictionWheelState;
RemoteSwitch_t g_switch1;
extern RampGen_t frictionRamp ;  //?????
extern RampGen_t LRSpeedRamp ;   //??????
extern RampGen_t FBSpeedRamp  ;   

float yawSpeedTarget = 0.0;
float yawAngleTarget = 0.0;
float pitchAngleTarget = 0.0;

//?????????
void RemoteTaskInit()
{
	frictionRamp.SetScale(&frictionRamp, FRICTION_RAMP_TICK_COUNT);
	frictionRamp.ResetCounter(&frictionRamp);
	
	yawSpeedTarget = 0.0;
	pitchAngleTarget = 0.0;
	/*???????*/
	ChassisSpeedRef.forward_back_ref = 0.0f;
	/*???*/
	FrictionWheelState = FRICTION_WHEEL_OFF;
}

//???????
void RemoteControlProcess(Remote *rc)
{
	if(WorkState == NORMAL_STATE)
	{
		ChassisSpeedRef.forward_back_ref = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT;
		//ChassisSpeedRef.left_right_ref   = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT; 
		bullet_ref = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT * 1.5;
		//bullet2_ref = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT * 1.5;
		//bullet_angle_target += (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_BULLET_POSITION_REF_FACT;
		//bullet2_angle_target += (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_BULLET_POSITION_REF_FACT;
		
		pitchAngleTarget += (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;
		yawSpeedTarget = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_YAW_SPEED_INC_FACT;
	}
	RemoteShootControl(&g_switch1, rc->s1);
}

//???????
void RemoteControlProcessAuto(Remote *rc)
{
	if(WorkState == DEFEND_STATE || WorkState == ATTACK_STATE)
	{
		bullet_ref = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT * 0.2;
		bullet2_ref = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT * 1.5;
		//bullet_angle_target += (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_BULLET_POSITION_REF_FACT;
		//bullet2_angle_target += (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_BULLET_POSITION_REF_FACT;
	}
	//RemoteShootControl(&g_switch1, rc->s1);
}

/*??????*/   
void GetRemoteSwitchAction(RemoteSwitch_t *sw, uint8_t val)
{
	static uint32_t switch_cnt = 0;

	sw->switch_value_raw = val;
	sw->switch_value_buf[sw->buf_index] = sw->switch_value_raw;

	//value1 value2????????
	//value1?4????0
	sw->switch_value1 = (sw->switch_value_buf[sw->buf_last_index] << 2)|
	(sw->switch_value_buf[sw->buf_index]);

	sw->buf_end_index = (sw->buf_index + 1)%REMOTE_SWITCH_VALUE_BUF_DEEP;

	sw->switch_value2 = (sw->switch_value_buf[sw->buf_end_index]<<4)|sw->switch_value1;	

	//????????,???????,????
	if(sw->switch_value_buf[sw->buf_index] == sw->switch_value_buf[sw->buf_last_index])
	{
		switch_cnt++;	
	}
	else
	{
		switch_cnt = 0;
	}
	//???????????,?????40??????,????????switch_long_value
	if(switch_cnt >= 40)
	{
		sw->switch_long_value = sw->switch_value_buf[sw->buf_index]; 	
	}
	//????????
	sw->buf_last_index = sw->buf_index;
	sw->buf_index++;		
	if(sw->buf_index == REMOTE_SWITCH_VALUE_BUF_DEEP)
	{
		sw->buf_index = 0;	
	}			
}


//???????
void RemoteDataPrcess(uint8_t *pData)
{
	if(pData == NULL)
	{
			return;
	}
	//??? 11*4 + 2*2 = 48,?? 6 Bytes
	//16?,???11?
	RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
	RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
											 ((int16_t)pData[4] << 10)) & 0x07FF;
	RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
	
	//16?,??????
	RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
	RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);

	//???? 8 Bytes
	RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
	RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
	RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);    

	RC_CtrlData.mouse.press_l = pData[12];
	RC_CtrlData.mouse.press_r = pData[13];
	
	//???? 2 Bytes = 16 bits ,????????
	RC_CtrlData.key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);

	//??????
	if(RC_CtrlData.rc.s2 == 1) inputmode = REMOTE_INPUT; 
	else if(RC_CtrlData.rc.s2 == 3) inputmode = AUTO; 
	else inputmode = STOP; 
	
	/*???????(RC_CtrlData.rc.s1)??*/	//?????????
	GetRemoteSwitchAction(&g_switch1, RC_CtrlData.rc.s1);
	
	switch(inputmode)
	{
		case REMOTE_INPUT:               
		{
			if(WorkState == NORMAL_STATE)
			{ 
				RemoteControlProcess(&(RC_CtrlData.rc));
			}
		}break;
		case AUTO:              
		{
			RemoteControlProcessAuto(&(RC_CtrlData.rc));
		}break;
		case STOP:               
		{
			 
		}break;
	}
}