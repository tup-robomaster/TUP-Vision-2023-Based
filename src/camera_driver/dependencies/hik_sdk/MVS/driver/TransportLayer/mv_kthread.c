
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <asm-generic/errno-base.h>

#include "MvErrorDefine.h"
#include "mv_kthread.h"

int MV_KThreadRun(MV_TASK* aTask, 
                     int (*threadfn)(void *data),
                     void *data,
                     const char name[])
{
    //va_list ap;
    if (NULL == threadfn)
    {
        return -EINVAL;
    }

    //va_start(ap, namefmt);
    aTask->pTask =  (void*)kthread_run(threadfn, data, name);
    if (IS_ERR(aTask->pTask)) {
        printk(KERN_INFO "create kthread failed!\n");
        return -ENOMEM;
    }
    
    //va_end(ap);

    return MV_OK;
}

int MV_KThreadBind(MV_TASK* aTask, unsigned int cpu)
{
    if (NULL == aTask)
    {
        return -EINVAL;
    }

    if (!IS_ERR(aTask->pTask))
    {
        kthread_bind((struct task_struct*)(aTask->pTask), cpu);
    }

    return MV_OK;
}

int MV_KTkthreadStop(MV_TASK* aTask)
{
    if (NULL == aTask)
    {
        return -EINVAL;
    }

    if (!IS_ERR(aTask->pTask))
    {
        return kthread_stop((struct task_struct*)(aTask->pTask));
    }
    return MV_OK;
}

int MV_KThreadShouldStop(void)
{
    return kthread_should_stop();
}
