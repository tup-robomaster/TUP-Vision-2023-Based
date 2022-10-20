
#include <linux/kernel.h>
#include <linux/delay.h>
#include <asm/current.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include "mv_misc.h"


/*******************************************************************************
@   函数原型：void MV_Printk(int res)
@   函数功能：打印函数
@   参数res： 打印值
@   返回值：无
*******************************************************************************/
void MV_Printk(const char* aFormat, ...)
{
    return;

    va_list lArgs;

    va_start(lArgs, aFormat);
    vprintk(aFormat, lArgs);
    va_end(lArgs);
}

/*******************************************************************************
@   函数原型：void MV_Mdelay(int nMSec)
@   函数功能：时间等待函数
@   参数nMSec： 等待的毫秒数
@   返回值：
*******************************************************************************/
void MV_Mdelay(int nMSec)
{
    mdelay(nMSec);
}

void* MV_GetCurrent(void)
{
    return (void*)current;
}


void* MV_GetCurrentMm(void)
{
    return (void*)current->mm;
}
