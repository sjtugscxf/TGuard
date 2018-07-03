#ifndef _CAN_BUS_TASK_H_
#define _CAN_BUS_TASK_H_

#include "main.h"

//RxID
#define CMFL_RXID 0x201u
#define CMFR_RXID 0x202u
#define BULLET_RXID 0x203u
#define BULLET2_RXID 0x204u

#define GMYAW_RXID 0x205u
#define GMPITCH_RXID 0x206u

#define UPMSG_RXID 0x305u

#define FRICL_RXID 0x201u
#define FRICR_RXID 0x202u

typedef struct{
	uint16_t angle;
	int16_t RotateSpeed;//RPM
}Motor820RRxMsg_t;

typedef struct{
	uint16_t angle;
	int16_t realIntensity;
	int16_t giveIntensity;
}Motor6623RxMsg_t;

extern Motor820RRxMsg_t CMFLRx;
extern Motor820RRxMsg_t CMFRRx;
extern Motor820RRxMsg_t BulletRx;
extern Motor820RRxMsg_t Bullet2Rx;
extern Motor6623RxMsg_t GMPITCHRx;
extern Motor6623RxMsg_t	GMYAWRx;
extern Motor820RRxMsg_t FRICLRx;
extern Motor820RRxMsg_t FRICRRx;

void CanReceiveMsgProcess(CanRxMsg * msg);
void FricCanReceiveMsgProcess(CanRxMsg * msg);
void Set_Gimbal_Current(CAN_TypeDef *CANx, int16_t gimbal_pitch_iq, int16_t gimbal_yaw_iq);
void Set_CM_Speed(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);

extern uint8_t redBuf;
extern uint8_t gameProgress;
extern uint8_t bulletFreq;
extern uint16_t shooterHeat0;
extern float bulletSpeed;
extern uint8_t bulletSpeedBuf[4];
#endif

