
#ifndef __MISC_H__
#define __MISC_H__


#ifdef __cplusplus
extern "C"
{
#endif 

#define MV_ACCEPT           1
#define MV_DROP             0

#define MV_TRUE 1
#define MV_FALSE 0

// #define MV_OK               0
// #define MV_ERROR            -1

void MV_Printk( const char* aFormat, ... );
extern void MV_Mdelay(int nMSec);
extern void* MV_GetCurrent(void);
extern void* MV_GetCurrentMm(void);

#ifdef __cplusplus
}
#endif 

#endif    //__MISC_H__
