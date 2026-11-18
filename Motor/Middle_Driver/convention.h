#ifndef __CONVENTION_H
#define __CONVENTION_H

// 0xlmn l表示报文是谁发的，0表示主控节点，1表示第一个受控节点，以此类推
//			 m表示谁来接收这串报文，0表示主控节点，1表示第一个受控节点，以此类推
//			 n用来匹配任务
#define CAN_ID_TARGET_SPEED 0x011 
#define CAN_ID_PID 0x012
#define CAN_ID_HEARTBEAT 0x013
#define CAN_ID_STATUS 0x100
#define CAN_DLC_TARGET_SPEED 2
#define CAN_DLC_PID 6
#define CAN_DLC_STATUS 4
#define CAN_DLC_HEARTBEAT 1
#define EMERGENCY_TIME 500

#include <stdint.h>

typedef struct
{
	uint16_t Can_Id;
	uint8_t Can_data[8];
} CAN_struct;

#endif
