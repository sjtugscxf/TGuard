#ifndef _CONTROL_TASK_H_
#define _CONTROL_TASK_H_
#include "main.h"

#define MINMAX(value, min, max) value = (value < min) ? min : (value > max ? max : value)
#define fw_PID_INIT(Kp, Ki, Kd, KpMax, KiMax, KdMax, OutputMax) { \
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ,\
	Kp, Ki, Kd, 0.0, 0.0, 0.0, \
	KpMax, KiMax, KdMax, 0.0, \
	OutputMax, \
	&fw_PID_Calc, &fw_PID_Reset \
};
typedef struct fw_PID_Regulator_t
{
	float target;
	float feedback;
	float errorCurr;
	float errorSum;
	uint16_t SumCount;
	float errorLast;
	float kp;
	float ki;
	float kd;
	float componentKp;
	float componentKi;
	float componentKd;
	float componentKpMax;
	float componentKiMax;
	float componentKdMax;
	float output;
	float outputMax;
	
	void (*Calc)(struct fw_PID_Regulator_t *pid);
	void (*Reset)(struct fw_PID_Regulator_t *pid);
}fw_PID_Regulator_t;

void fw_PID_Reset(fw_PID_Regulator_t *pid);
void fw_PID_Calc(fw_PID_Regulator_t *pid);

int16_t ProcessYawPID(float target,  float velocity_feedback);
int16_t ProcessPitchPID(float target, float position_feedback, float velocity_feedback);
int16_t PID_PROCESS_Double(fw_PID_Regulator_t pid_position,fw_PID_Regulator_t pid_speed,
                            float target, float position_feedback, float velocity_feedback);
int16_t PID_PROCESS_Speed(fw_PID_Regulator_t pid_speed,float target, float velocity_feedback);

#define AUTO_ATTACK_YAW_KP      0.1f
#define AUTO_ATTACK_YAW_KD      0 
#define AUTO_ATTACK_PITCH_KP      0.0005f
#define AUTO_ATTACK_PITCH_KD      0 
#define YAW_OFFSET         330u  
#define PITCH_OFFSET       220u  
#define CHASSIS_SPEED_ATTENUATION   (1.30f)
#define CHASSIS_MOTOR_ROTATE_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	1.4f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	5000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}

#define CHASSIS_MOTOR_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	6.5f,\
	0.0f,\
	1.0f,\
	0,\
	0,\
	0,\
	4900,\
	3500,\
	1500,\
	0,\
	4950,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}

#define BULLET_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	8.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	10000,\
	10000,\
	10000,\
	0,\
	10000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}

#define BULLET_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	2.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	10000,\
	10000,\
	10000,\
	0,\
	4900,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}

typedef enum
{
	START_STATE,
	PREPARE_STATE,     	
	NORMAL_STATE,		  
  DEFEND_STATE,
  ATTACK_STATE,  
	STOP_STATE        
}WorkState_e;

extern WorkState_e WorkState;
extern uint8_t find_enemy;
extern uint16_t enemy_pitch;
extern uint16_t enemy_yaw;
extern uint16_t enemy_detect_cnt;

extern int16_t yawIntensity ;

extern uint16_t maincnt;

extern fw_PID_Regulator_t pitchPositionPID;
extern fw_PID_Regulator_t yawPositionPID;
extern fw_PID_Regulator_t pitchSpeedPID;
extern fw_PID_Regulator_t yawSpeedPID;

extern double bullet_angle_target;
extern double bullet2_angle_target;
extern float yawRealAngle;
extern float pitchRealAngle;
extern float auto_attack_yaw_kp;
extern float auto_attack_pitch_kp;
extern float auto_attack_yaw_kd;
extern float auto_attack_pitch_kd;

void CMControlInit(void);
#endif

