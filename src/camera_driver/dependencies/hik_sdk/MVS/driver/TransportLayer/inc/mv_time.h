
#ifndef __TIME_H__
#define __TIME_H__

#include "mv_misc.h"

#pragma pack( push, 1 )
typedef struct {
		int tm_sec;
		int tm_min;
		int tm_hour;
		int tm_mday;
		int tm_mon;
		int tm_year;
		int tm_wday;
		int tm_yday;
		int tm_isdst;
}MV_TM_RtcTime;
#pragma pack( pop )

extern void MV_MSleep(unsigned int msecs);
extern void MV_MSleepInterruptible(unsigned int msecs);

extern MV_TM_RtcTime TM_GetCurrentTime(void);
extern unsigned long long TM_GetTickCount(void);

#endif	//__TIME_H__

