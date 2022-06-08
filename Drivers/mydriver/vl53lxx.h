#ifndef __VL53LXX_H
#define __VL53LXX_H
#include "sys.h"
#include "vl53l0x.h"
#include "vl53l1x.h"

extern u16 vl53lxxId;	/*vl53оƬID*/
extern bool isEnableVl53lxx;
//extern zRange_t vl53lxx;

void vl53l1x(void);
void vl53lxxInit(void);
//bool vl53lxxReadRange(zRange_t* zrange);
void setVl53lxxState(u8 enable);
u16 LaserGetHeight(void);

#endif /* __VL53LXX_H */

