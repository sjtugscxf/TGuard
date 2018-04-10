#ifndef _CAN_BUS_TASK_H_
#define _CAN_BUS_TASK_H_

#include "main.h"

//RxID
#define CMFL_RXID 0x201u
#define CMFR_RXID 0x202u
#define BULLET_RXID 0x203u
#define BULLET2_RXID 0x204u

#define GMYAW_RXID 0x206u
#define GMPITCH_RXID 0x205u

#define UPMSG_RXID 0x301u

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


void CanReceiveMsgProcess(CanRxMsg * msg);
void Set_Gimbal_Current(CAN_TypeDef *CANx, int16_t gimbal_pitch_iq, int16_t gimbal_yaw_iq);
void Set_CM_Speed(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
#endif

