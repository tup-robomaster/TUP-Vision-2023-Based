
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/version.h>

#include "mv_lock.h"
#include "MvErrorDefine.h"
#include <asm-generic/errno.h>
#include <asm-generic/errno-base.h>

/*******************************************************************************
@   函数原型：int MV_LockCheck(void)
@   函数功能：自旋锁长度验证
@   参数： 无
@   返回值：成功MV_OK 失败MV_E_PARAMETER
*******************************************************************************/
int MV_LockCheck(void)
{
    //if(MV_SPINLOCK_SIZE < sizeof(spinlock_t))
    //{
    //    printk("spinlock_t size = %d / %d\n", (int)sizeof(spinlock_t), (int)MV_SPINLOCK_SIZE);
    //    printk("Lock not compatible with your distribution\n");
    //    return MV_E_PARAMETER;
    //}

    return MV_OK;
}

/*******************************************************************************
@   函数原型：int MV_LockInit(MV_LOCK* aLock)
@   函数功能：自旋锁初始化
@   参数aLock： 自旋锁
@   返回值：成功MV_OK 失败MV_E_PARAMETER
*******************************************************************************/
int MV_LockInit(MV_LOCK* aLock)
{
    spinlock_t* lSpinlock = NULL;

    if(NULL == aLock)
    {
        return -EINVAL;
    }

    aLock->pSpinLock = kmalloc(sizeof(spinlock_t), GFP_KERNEL);
    if (NULL == aLock->pSpinLock)
    {
        return -ENOMEM;
    }
    
    lSpinlock = (spinlock_t *)(aLock->pSpinLock);

    spin_lock_init(lSpinlock);

    return MV_OK;
} 

int MV_LockFree(MV_LOCK* aLock)
{
    if (aLock == NULL)
    {
        return -EINVAL;
    }

    if (NULL != aLock->pSpinLock)
    {
        kfree((spinlock_t *)aLock->pSpinLock);
        aLock->pSpinLock = NULL;
    }

    return MV_OK;
}

/*******************************************************************************
@   函数原型：int MV_LockAcquire(MV_LOCK* aLock, unsigned long *aFlag)
@   函数功能：自旋锁加锁
@   参数aLock： 自旋锁
@   参数aFlag： 中断标志
@   返回值：成功MV_OK 失败MV_E_PARAMETER
*******************************************************************************/
int MV_LockAcquire(MV_LOCK* aLock)
{
    spinlock_t* lSpinlock = NULL;

    if(NULL == aLock)
    {
        return -EINVAL;
    }

    lSpinlock = (spinlock_t *)(aLock->pSpinLock);

    spin_lock( lSpinlock );

    return MV_OK;
}

int MV_LockIrqAcquire(MV_LOCK* aLock)
{
    spinlock_t* lSpinlock = NULL;
    if(NULL == aLock)
    {
        return -EINVAL;
    }
    lSpinlock = (spinlock_t *)(aLock->pSpinLock);

    spin_lock_irq( lSpinlock);

    return MV_OK;
}

int MV_LockBhAcquire(MV_LOCK* aLock)
{
    spinlock_t* lSpinlock = NULL;

    if(NULL == aLock)
    {
        return -EINVAL;
    }
    lSpinlock = (spinlock_t *)(aLock->pSpinLock);

    spin_lock_bh( lSpinlock);

    return MV_OK;
}


/*******************************************************************************
@   函数原型：int MV_LockRelease(MV_LOCK* aLock, unsigned long *aFlag)
@   函数功能：自旋锁解锁
@   参数aLock： 自旋锁
@   参数aFlag： 中断标志
@   返回值：成功MV_OK 失败MV_E_PARAMETER
*******************************************************************************/
int MV_LockRelease(MV_LOCK* aLock)
{
    spinlock_t* lSpinlock = NULL;

    if(NULL == aLock)
    {
        return -EINVAL;
    }
    lSpinlock = (spinlock_t *)(aLock->pSpinLock);

    spin_unlock( lSpinlock );

    return MV_OK;
}

int MV_LockIrqRelease(MV_LOCK* aLock)
{
    spinlock_t* lSpinlock = NULL;

    if(NULL == aLock)
    {
        return -EINVAL;
    }
    lSpinlock = (spinlock_t *)(aLock->pSpinLock);

    spin_unlock_irq( lSpinlock );

    return MV_OK;
}

int MV_LockBhRelease(MV_LOCK* aLock)
{
    spinlock_t* lSpinlock = NULL;

    if(NULL == aLock)
    {
        return -EINVAL;
    }
    lSpinlock = (spinlock_t *)(aLock->pSpinLock);

    spin_unlock_bh( lSpinlock );

    return MV_OK;
}

