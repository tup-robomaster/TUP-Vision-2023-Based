
#include <linux/time.h>
#include <linux/timex.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/math64.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,0,0)
#include <linux/efi.h>
#else
#include <linux/rtc.h>
#endif

#include "mv_time.h"

/** 
 * @fn		void MV_MSleep(unsigned int msecs)
 * @brief	睡眠时间函数
 * @param	msecs: 睡眠的毫秒数
 * @return	无
 */ 
void MV_MSleep(unsigned int msecs)
{
    msleep(msecs);
}

/** 
 * @fn		void MV_MSleepInterruptible(unsigned int msecs)
 * @brief	可中断的睡眠时间函数
 * @param	msecs: 睡眠的毫秒数
 * @return	无
 */ 
void MV_MSleepInterruptible(unsigned int msecs)
{
    msleep_interruptible(msecs);
}

/*******************************************************************************
@   函数原型：TM_RtcTime TM_GetCurrentTime(void)
@   函数功能：获取本地时间
@   参数： 无
@   返回值：本地时间
*******************************************************************************/
MV_TM_RtcTime TM_GetCurrentTime(void)
{
    struct rtc_time tm         = { 0 };
    MV_TM_RtcTime aCurrentTime = { 0 };

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,0,0)
    struct timespec64 txc = { 0 };

    /* 获取当前的UTC时间 */
    ktime_get_real_ts64(&txc);

    /* 把UTC时间调整为本地时间 */
    txc.tv_sec -= sys_tz.tz_minuteswest * 60;
    /* 算出时间中的年月日等数值到tm中 */
    rtc_time64_to_tm(txc.tv_sec,&tm);

#else
    struct timex  txc = { 0 };
    /* 获取当前的UTC时间 */
    do_gettimeofday(&(txc.time));

    /* 把UTC时间调整为本地时间 */
    txc.time.tv_sec -= sys_tz.tz_minuteswest * 60;
    /* 算出时间中的年月日等数值到tm中 */
    rtc_time_to_tm(txc.time.tv_sec,&tm);
#endif
    tm.tm_year = tm.tm_year + 1900;
    tm.tm_mon = tm.tm_mon + 1;

    aCurrentTime = *((MV_TM_RtcTime*)&tm);

    return aCurrentTime;
}



/*******************************************************************************
@   函数原型：unsigned long TM_GetTickCount(void)
@   函数功能：获得系统当前时间的毫秒数
@   参数： 无
@   返回值：系统当前时间的毫秒数
*******************************************************************************/
unsigned long long TM_GetTickCount(void)
{
    unsigned long long msce = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,0,0)
    struct timespec64 txc = { 0 };
    unsigned long long tv_sec = 0;
    unsigned long long tv_nsec = 0;
    /* 获取当前的UTC时间 */
    ktime_get_real_ts64(&txc);
    tv_sec = (unsigned long long)(txc.tv_sec);
    tv_nsec = (unsigned long long)(txc.tv_nsec);
    msce = txc.tv_sec * 1000 + div_u64(tv_nsec, 1000000);
#else
    struct  timeval   tv;
    unsigned long long tv_sec = 0;
    unsigned long long tv_usec = 0;
    do_gettimeofday(&tv);
    tv_sec = (unsigned long long)(tv.tv_sec);
    tv_usec = (unsigned long long)(tv.tv_usec);
    msce = tv_sec * 1000 + div_u64(tv_usec, 1000);
#endif
    return  msce;
}

