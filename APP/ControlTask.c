#include "main.h"

void fw_PID_Reset(fw_PID_Regulator_t *pid){
	
}

void fw_PID_Calc(fw_PID_Regulator_t *pid){
	pid->errorCurr = pid->target - pid->feedback;
	if(pid->SumCount <=500){
		pid->errorSum += pid->target - pid->feedback;
		pid->SumCount++;
		}
	else {
	  pid->errorSum = 0;
		pid->SumCount = 0;
	}
	pid->componentKp = pid->kp * pid->errorCurr;
	MINMAX(pid->componentKp, -pid->componentKpMax, pid->componentKpMax);
	pid->componentKi = pid->ki * pid->errorSum;
	MINMAX(pid->componentKi, -pid->componentKiMax, pid->componentKiMax);
	pid->componentKd = pid->kd * (pid->errorCurr - pid->errorLast);
	MINMAX(pid->componentKd, -pid->componentKdMax, pid->componentKdMax);
	
	pid->errorLast = pid->errorCurr;
	
	pid->output = pid->componentKp + pid->componentKi + pid->componentKd;
	MINMAX(pid->output, -pid->outputMax, pid->outputMax);
}

extern fw_PID_Regulator_t pitchPositionPID;
extern fw_PID_Regulator_t yawPositionPID;
extern fw_PID_Regulator_t pitchSpeedPID;
extern fw_PID_Regulator_t yawSpeedPID;

int16_t ProcessYawPID(float target, float velocity_feedback)
{
	return PID_PROCESS_Speed(yawSpeedPID,target,velocity_feedback);
}
int16_t ProcessPitchPID(float target, float position_feedback, float velocity_feedback)
{
	return PID_PROCESS_Double(pitchPositionPID,pitchSpeedPID,target,position_feedback,velocity_feedback);
}
int16_t PID_PROCESS_Double(fw_PID_Regulator_t pid_position,fw_PID_Regulator_t pid_speed,float target, float position_feedback, float velocity_feedback)
{		
	pid_position.target = target;
	pid_position.feedback = position_feedback;
	pid_position.Calc(&pid_position);
	
	pid_speed.target = pid_position.output;
	pid_speed.feedback = velocity_feedback;
	pid_speed.Calc(&pid_speed);
	return pid_speed.output;
}

int16_t PID_PROCESS_Speed(fw_PID_Regulator_t pid_speed,float target, float velocity_feedback)
{
	pid_speed.target = target;
	pid_speed.feedback = velocity_feedback;
	pid_speed.Calc(&pid_speed);
	return pid_speed.output;
}

uint8_t find_enemy = 0;
uint8_t on_enemy = 0;
uint8_t target_hero = 0;
uint16_t yaw_offset = 320;
uint16_t pitch_offset = 210;
uint16_t enemy_yaw = 320;
uint16_t enemy_pitch = 210;
uint16_t manifold_fine_cnt = 0;
WorkState_e WorkState = START_STATE;
uint16_t prepare_time = 0;

uint8_t bulletshooted = 0;
uint16_t bulletshootedcnt = 0;
uint8_t nobullet = 0;
uint8_t bulletSpeedBuf_last[4] = {0};

PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; 
PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t BulletSpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t Bullet2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;

int16_t CMFLIntensity = 0, CMFRIntensity = 0, BulletIntensity = 0,Bullet2Intensity = 0;
int16_t yawIntensity = 0;		
int16_t pitchIntensity = 0;

void CMControlInit(void)
{
	CMRotatePID.Reset(&CMRotatePID);
	CM1SpeedPID.Reset(&CM1SpeedPID);
	CM2SpeedPID.Reset(&CM2SpeedPID);
	BulletSpeedPID.Reset(&BulletSpeedPID);
	Bullet2SpeedPID.Reset(&Bullet2SpeedPID);
}

void ControlCMFL(void)
{		
	CM1SpeedPID.ref =  ChassisSpeedRef.forward_back_ref*0.075+ ChassisSpeedRef.left_right_ref*0.075;
	CM1SpeedPID.ref = 160 * CM1SpeedPID.ref;	
			
	CM1SpeedPID.fdb = CMFLRx.RotateSpeed;

	CM1SpeedPID.Calc(&CM1SpeedPID);
	CMFLIntensity = CHASSIS_SPEED_ATTENUATION * CM1SpeedPID.output;
}

void ControlCMFR(void)
{		
	CM2SpeedPID.ref = - ChassisSpeedRef.forward_back_ref*0.075 + ChassisSpeedRef.left_right_ref*0.075;
	CM2SpeedPID.ref = 160 * CM2SpeedPID.ref;	
			
	CM2SpeedPID.fdb = CMFRRx.RotateSpeed;

	CM2SpeedPID.Calc(&CM2SpeedPID);
	CMFRIntensity = CHASSIS_SPEED_ATTENUATION * CM2SpeedPID.output;
}

void ControlBullet(void)
{		
	BulletSpeedPID.kp = 3.0;
	BulletSpeedPID.kd = 0.0;
	BulletSpeedPID.ref = bullet_ref*0.075;
	BulletSpeedPID.ref = 160 * BulletSpeedPID.ref;	
			
	BulletSpeedPID.fdb = BulletRx.RotateSpeed;

	BulletSpeedPID.Calc(&BulletSpeedPID);
	BulletIntensity = CHASSIS_SPEED_ATTENUATION * BulletSpeedPID.output;
}

unsigned char testred1 = 0;
unsigned char testred2 = 0;
unsigned char testred3 = 0;
unsigned char testred4 = 0;
unsigned char calicnt = 0;
unsigned int downcnt = 0;
unsigned int upcnt = 0;

float odometry = 0.0;
float odometryatt = 0.0;
void odometryLoop()
{
	testred1 = redBuf & 0x01;
	testred2 = (redBuf & 0x02)>>1;
	testred3 = (redBuf & 0x04)>>2;
	testred4 = (redBuf & 0x08)>>3;
	
	if(testred1 == 0)
	{
		if(calicnt>3) odometry = 0.0;
		else calicnt++;
	}
	else calicnt = 0;
	
	if(testred2 == 0)
	{
		if(downcnt>3) odometry = -500000.0;
		else downcnt++;
	}
	else downcnt = 0;
	
	if(testred4 == 0)
	{
		if(upcnt>3) odometry = 500000.0;
		else upcnt++;
	}
	else upcnt = 0;
	
	if((CMFLRx.RotateSpeed > 50 || CMFLRx.RotateSpeed < -50) && (CMFRRx.RotateSpeed > 50 || CMFRRx.RotateSpeed < -50))
	{
		odometry += (CMFLRx.RotateSpeed - CMFRRx.RotateSpeed) * ODOMETRY_FACT;
	}
}

extern FrictionWheelState_e FrictionWheelState;
extern Shoot_State_e ShootState;
unsigned int enemy_lost = 0;
uint8_t enemy_far = 0;
void WorkStateFSM(void)
{
	static int blink_cnt = 0;
	blink_cnt++;
	switch (WorkState)
	{
		case START_STATE:
		{
			if(prepare_time<2000) prepare_time++;
			if(prepare_time == 2000)
			{
				WorkState = PREPARE_STATE;
			}
			if (inputmode == STOP) WorkState = STOP_STATE;
		}break;
		case PREPARE_STATE:
		{
			if(prepare_time<4000) prepare_time++;
			if(prepare_time == 4000)
			{
				WorkState = NORMAL_STATE;
				prepare_time = 0;
				GREEN_LED_OFF();
				RED_LED_OFF();
				blink_cnt = 0;
			}
			
			if (inputmode == STOP) WorkState = STOP_STATE;
		}break;
		case NORMAL_STATE:
		{
			if (inputmode == STOP) 
			{
				WorkState = STOP_STATE;
				SetFrictionWheelSpeed(1000); 
				frictionRamp.ResetCounter(&frictionRamp);
				FrictionWheelState = FRICTION_WHEEL_OFF;
			}
			else if (inputmode == AUTO)
			{
				WorkState = DEFEND_STATE;
			}
			
			if(gameProgress == 4)
			{
				SetFrictionWheelSpeed(1000 + (FRICTION_WHEEL_MAX_DUTY-1000)*frictionRamp.Calc(&frictionRamp)); 
				if(frictionRamp.IsOverflow(&frictionRamp))
				{
					WorkState = DEFEND_STATE;
					FrictionWheelState = FRICTION_WHEEL_ON;
				}
			}
			
			if (blink_cnt == 1000) 
			{
				blink_cnt = 0;
				GREEN_LED_TOGGLE();
			}
		}break;
		case DEFEND_STATE:  
		{
			if (find_enemy == 1 && nobullet == 0 && enemy_far == 0) 
			{
				WorkState = ATTACK_STATE;
				odometryatt = odometry;
			}
			
			if (inputmode == STOP) 
			{
				WorkState = STOP_STATE;
				SetFrictionWheelSpeed(1000); 
				FrictionWheelState = FRICTION_WHEEL_OFF;
				frictionRamp.ResetCounter(&frictionRamp);
				bulletshootedcnt = 0;
				nobullet = 0;
				find_enemy = 0;
				enemy_yaw = 320;
				enemy_pitch = 210;
				bullet_ref = 0;
			}

			if (blink_cnt == 1000) 
			{
				blink_cnt = 0;
				RED_LED_TOGGLE();
			}
		}break;
		case ATTACK_STATE: 
		{
			if (nobullet == 1)  WorkState = DEFEND_STATE;
			
			if(manifold_fine_cnt>500)   
			{
				WorkState = DEFEND_STATE;
			}
			else
			{
				manifold_fine_cnt++;
			}
			
			if (find_enemy == 0 || enemy_far == 1) 
			{
				enemy_lost++;
				if (enemy_lost > 1200) 
				{
					WorkState = DEFEND_STATE;
					enemy_lost = 0;
				}
			}
			else enemy_lost = 0;
			
			if (inputmode == STOP) 
			{
				WorkState = STOP_STATE;
				SetFrictionWheelSpeed(1000); 
				frictionRamp.ResetCounter(&frictionRamp);
				FrictionWheelState = FRICTION_WHEEL_OFF;
				bulletshootedcnt = 0;
				nobullet = 0;
				find_enemy = 0;
				enemy_yaw = 320;
				enemy_pitch = 210;
				bullet_ref = 0;
			}

			if (blink_cnt == 1000) 
			{
				blink_cnt = 0;
			}
		}break;
		case STOP_STATE:
		{
			if (inputmode == REMOTE_INPUT)
			{
				WorkState = PREPARE_STATE;
				RemoteTaskInit();
			}
		}break;
	}
}

void setCMMotor()
{
	Set_CM_Speed(CAN2, CMFLIntensity, CMFRIntensity, BulletIntensity, Bullet2Intensity);		 
}

void setGMMotor()
{
	Set_Gimbal_Current(CAN2, pitchIntensity, yawIntensity); 
}

#define NORMALIZE_ANGLE180(angle) angle = ((angle) > 180) ? ((angle) - 360) : (((angle) < -180) ? (angle) + 360 : angle)
fw_PID_Regulator_t pitchPositionPID = fw_PID_INIT(7.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t yawPositionPID = fw_PID_INIT(5.0, 0.0, 0.5, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t pitchSpeedPID = fw_PID_INIT(20.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 3500.0); //KP²»ÄÜ³¬¹ý30
fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(10.0, 0.0, 0, 10000.0, 10000.0, 10000.0, 2000.0);
#define yaw_zero 7200  
#define pitch_zero 5796
float yawRealAngle = 0.0;
float pitchRealAngle = 0.0;
float gap_angle = 0.0;

void ControlYawSpeed(void)
{	
	yawIntensity = ProcessYawPID(yawSpeedTarget,-MPU6050_Real_Data.Gyro_Z);
}

void ControlPitch(void)
{
	uint16_t pitchZeroAngle = pitch_zero;
				
	pitchRealAngle = -(GMPITCHRx.angle - pitchZeroAngle) * 360 / 8192.0;
	NORMALIZE_ANGLE180(pitchRealAngle);

	MINMAX(pitchAngleTarget, PITCHANGLETARGETMIN1, PITCHANGLETARGETMAX1);
				
	pitchIntensity = -ProcessPitchPID(pitchAngleTarget,pitchRealAngle,-MPU6050_Real_Data.Gyro_X);
}

float enemy_yaw_err = 0;
float enemy_yaw_out = 0;
float enemy_pitch_err = 0;
float enemy_pitch_out = 0;

uint8_t pitch_dir = 0;
uint8_t up_dir = 0;

float pitchAngleTargetMin = 0;
float yawSpeed = 200;
void Defend_Action()
{
	if(odometry > -170000 && odometry < -140000)
	{
		pitchAngleTargetMin = PITCHANGLETARGETMIN2;
	}
	else 
	{
		pitchAngleTargetMin = PITCHANGLETARGETMIN1;
	}
	
	if(pitchAngleTarget > (PITCHANGLETARGETMAX1 - 2.0))
	{
		pitch_dir = 0;
	}
	else if(pitchAngleTarget < (pitchAngleTargetMin + 2.0))
	{
		pitch_dir = 1;
	}
	
	if (pitch_dir == 0)
	{
		pitchAngleTarget -= PITCH_DEFEND_SPEED;
	}
	else if (pitch_dir == 1)
	{
		pitchAngleTarget += PITCH_DEFEND_SPEED;
	}
	
	yawSpeedTarget = YAW_DEFEND_SPEED;
	
	if(odometry < ODOMETRY_DOWNMAX1) up_dir = 0;
	if(odometry > ODOMETRY_UPMAX1) up_dir = 1;
	
	if(odometry > -170000 && odometry < -140000 && nobullet == 0)
	{
		if(up_dir == 0) 
		{
			if (ChassisSpeedRef.forward_back_ref > ODOMETRY_SPEED3) 
			{
				ChassisSpeedRef.forward_back_ref -= 4;
			}
			else ChassisSpeedRef.forward_back_ref = ODOMETRY_SPEED3;
		}
		else if(up_dir == 1) 
		{
			if (ChassisSpeedRef.forward_back_ref < -ODOMETRY_SPEED3)
			{
				ChassisSpeedRef.forward_back_ref += 4;
			}
			else ChassisSpeedRef.forward_back_ref = -ODOMETRY_SPEED3;
		}
	}
	else 
	{
		if(up_dir == 0) 
		{
			if (ChassisSpeedRef.forward_back_ref < ODOMETRY_SPEED1) 
			{
				if(odometry < -400000) ChassisSpeedRef.forward_back_ref += 10;
				else ChassisSpeedRef.forward_back_ref += 0.4;
			}
			else ChassisSpeedRef.forward_back_ref = ODOMETRY_SPEED1;
		}
		else if(up_dir == 1) 
		{
			if (ChassisSpeedRef.forward_back_ref > -ODOMETRY_SPEED1)
			{
				if(odometry > 400000) ChassisSpeedRef.forward_back_ref -= 10;
				else ChassisSpeedRef.forward_back_ref -= 0.4;
			}
			else ChassisSpeedRef.forward_back_ref = -ODOMETRY_SPEED1;
		}
	}
	
	bullet_ref = 0;
}
uint8_t catchedcnt =  0 ;
uint8_t up_diratt = 0;
float pitchAngleTargetMax = PITCHANGLETARGETMAX1;
void Attack_Action()
{
	if(odometry < (odometryatt + ODOMETRY_DOWNMAX2) || odometry < ODOMETRY_DOWNMAX2) ChassisSpeedRef.forward_back_ref = ODOMETRY_SPEED2;
	if(odometry > (odometryatt + ODOMETRY_UPMAX2) || odometry > ODOMETRY_UPMAX2) ChassisSpeedRef.forward_back_ref = -ODOMETRY_SPEED2;
	
//	if(shooterHeat0 > STOPHEAT) 
//	{
//		if(odometry < (odometryatt + ODOMETRY_DOWNMAX2) || odometry < ODOMETRY_DOWNMAX1) up_diratt = 0;
//		if(odometry > (odometryatt + ODOMETRY_UPMAX2) || odometry > ODOMETRY_UPMAX1) up_diratt = 1;
//		
//		if(up_diratt == 0) 
//		{
//			ChassisSpeedRef.forward_back_ref = ODOMETRY_SPEED2;
//		}
//		else if(up_diratt == 1) 
//		{
//			ChassisSpeedRef.forward_back_ref = -ODOMETRY_SPEED2;
//		}
//	}
//	else if (shooterHeat0 < 150) 
	ChassisSpeedRef.forward_back_ref = 0.0;
		
	static float enemy_yaw_err_last = 0;
	enemy_yaw_err = (float)(yaw_offset - enemy_yaw);
	enemy_yaw_out = enemy_yaw_err *  AUTO_ATTACK_YAW_KP + (enemy_yaw_err - enemy_yaw_err_last)*AUTO_ATTACK_YAW_KD;
	if (enemy_yaw_out>100) enemy_yaw_out = 100;
	else if (enemy_yaw_out<-100) enemy_yaw_out = -100;
	yawSpeedTarget = enemy_yaw_out;
	enemy_yaw_err_last = enemy_yaw_err;
		
	static float enemy_pitch_err_last = 0;
	enemy_pitch_err = (float)(pitch_offset - enemy_pitch);
	enemy_pitch_out = enemy_pitch_err * AUTO_ATTACK_PITCH_KP + (enemy_pitch_err - enemy_pitch_err_last)*AUTO_ATTACK_PITCH_KD;
	if (enemy_pitch_out>1) enemy_pitch_out = 1;
	else if (enemy_pitch_out<-1) enemy_pitch_out = -1;
	pitchAngleTarget -= enemy_pitch_out;
	enemy_pitch_err_last = enemy_pitch_err;
	if(target_hero == 0) 
	{
		if (pitchAngleTarget > PITCHANGLETARGETMAX2) pitchAngleTarget = PITCHANGLETARGETMAX2;
	}
		
	if(enemy_yaw_err<50 && enemy_yaw_err>-50 && enemy_pitch_err<50 && enemy_pitch_err>-50) 
	{
		if (catchedcnt > 10)
		{
			if(shooterHeat0 < 100) bullet_ref = BULLET_SPEED; 
		}
		else catchedcnt++;
	}
	else 
	{
		catchedcnt = 0;
		bullet_ref = 0;
	}
}

void controlLoop()
{
	if(target_hero == 0) pitchAngleTargetMax = PITCHANGLETARGETMAX2;
	else pitchAngleTargetMax = PITCHANGLETARGETMAX1;
	
	if((enemy_pitch > ((pitchAngleTargetMax-pitchAngleTarget) * 3 + 310)) && pitchAngleTarget >20) enemy_far = 1;
	else enemy_far = 0;
	
	if(target_hero == 0) 
	{
		if(pitchAngleTarget < 22) pitch_offset = 210;
		else if(pitchAngleTarget < 37) pitch_offset = 206;
		else if(pitchAngleTarget < 50) pitch_offset = (unsigned int)(206 - (pitchAngleTarget - 37));
		else if(pitchAngleTarget < 55) pitch_offset = (unsigned int)(193 - (pitchAngleTarget - 50));
		else if(pitchAngleTarget < (PITCHANGLETARGETMAX1 + 0.1)) pitch_offset = (unsigned int)(188 - (pitchAngleTarget - 55));
	}
	else pitch_offset = 210;
		
	WorkStateFSM();
	
	if(WorkState != START_STATE) 
	{
		odometryLoop();
		
		if(bulletSpeedBuf[0] != bulletSpeedBuf_last[0] || bulletSpeedBuf[1] != bulletSpeedBuf_last[1] || bulletSpeedBuf[2] != bulletSpeedBuf_last[2] || bulletSpeedBuf[3] != bulletSpeedBuf_last[3])
		{
			bulletshooted = 1;
			bulletshootedcnt = 0;
			nobullet = 0;
		}
		else bulletshooted = 0;
		bulletSpeedBuf_last[0] = bulletSpeedBuf[0];
		bulletSpeedBuf_last[1] = bulletSpeedBuf[1];
		bulletSpeedBuf_last[2] = bulletSpeedBuf[2];
		bulletSpeedBuf_last[3] = bulletSpeedBuf[3];
		
		if(WorkState == DEFEND_STATE) Defend_Action();
		else if(WorkState == ATTACK_STATE) Attack_Action();
		
		ControlYawSpeed();
		ControlPitch();
		
		if(WorkState == STOP_STATE)
		{
			yawIntensity = 0;
			pitchIntensity = 0;
		}
		setGMMotor();
		
		ControlCMFL();
		ControlCMFR();
		
		if(shooterHeat0 > STOPHEAT) bullet_ref = 0;
		ControlBullet();
		
		if (bullet_ref > 100) 
		{
			if(bulletshootedcnt < 20000) bulletshootedcnt++;
			else nobullet = 1;
		}
		
		if(WorkState == STOP_STATE)
		{
			CMFLIntensity = 0;
			CMFRIntensity = 0;
			BulletIntensity = 0;
			Bullet2Intensity = 0;
		}
		setCMMotor();
	}
}