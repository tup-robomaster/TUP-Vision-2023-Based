
#include "mv_event.h"
#include "mv_result.h"
#include "mv_misc.h"


#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include "MvErrorDefine.h"
#include <asm-generic/errno.h>
#include <asm-generic/errno-base.h>


int MV_EventInit( MV_EVENT* aEvent )
{
    wait_queue_head_t* lWaitQueue = NULL;

    if (NULL == aEvent)
    {
        return -EINVAL;
    }

    aEvent->pEvent = kmalloc(sizeof(wait_queue_head_t), GFP_KERNEL);
    if (NULL == aEvent->pEvent)
    {
        return -ENOMEM;
    }
    
    lWaitQueue = ( wait_queue_head_t* ) aEvent->pEvent;
    
    memset(lWaitQueue, 0, sizeof(wait_queue_head_t));
    // MV_ASSERT( MV_EVENT_SIZE >= sizeof( wait_queue_head_t ) );

    aEvent->Flag = 0;
    init_waitqueue_head( lWaitQueue );

    //printk("end MV_EventInit\n");
    return MV_OK;
}

int MV_EventFree( MV_EVENT* aEvent )
{
    //MV_UNREFERENCED_PARAMETER( aEvent );
    if (NULL == aEvent)
    {
        return -EINVAL;
    }
    if (NULL != aEvent->pEvent)
    {
        kfree(aEvent->pEvent);
        aEvent->pEvent = NULL;
    }
    
    return MV_OK;
}

int MV_EventWait( MV_EVENT* aEvent )
{
    wait_queue_head_t* lWaitQueue = NULL;
    long lReturn                  = 0;

    lWaitQueue = (wait_queue_head_t*)aEvent->pEvent;
    lReturn = wait_event_interruptible( ( *lWaitQueue ), aEvent->Flag );
    aEvent->Flag = 0;
    return lReturn;

}

int MV_EventWaitWithTimeout( MV_EVENT* aEvent, unsigned long long aTimeout )
{
    long lReturn = MV_OK;
    wait_queue_head_t* lWaitQueue = ( wait_queue_head_t* ) aEvent->pEvent;
    
    lReturn = wait_event_interruptible_timeout( ( *lWaitQueue ), aEvent->Flag, 
        usecs_to_jiffies( aTimeout ) );
    aEvent->Flag = 0;
    return lReturn;

}

int MV_EventRaise( MV_EVENT* aEvent )
{
    wait_queue_head_t* lWaitQueue = ( wait_queue_head_t* ) aEvent->pEvent;

    aEvent->Flag = 1;
    wake_up_interruptible( lWaitQueue );

    return MV_OK;
}

int MV_EventReset( MV_EVENT* aEvent )
{
    aEvent->Flag = 0;

    return MV_OK;
}
