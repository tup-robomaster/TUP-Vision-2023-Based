
#include <linux/device.h>
#include <linux/module.h>
#include <linux/version.h>

#include "mv_device.h"
#include "MvErrorDefine.h"
#include <asm-generic/errno.h>
#include <asm-generic/errno-base.h>


int MV_ClassCreate(void** ppOutClass, const char* name)
{
    if (NULL == name)
    {
        return -EINVAL;
    }
    
    *ppOutClass = class_create(THIS_MODULE, name);
    if(IS_ERR(*ppOutClass))
    {
        return -EINVAL;
    }
 
    return MV_OK;
}

int MV_DeviceCreate(void** ppOutDevice, void** ppcls, const void* pDeviceParent, const void* pDev, const char *fmt, ...)
{

    va_list vargs;
    dev_t dev;

    if (NULL == ppOutDevice || NULL == ppcls || NULL == *ppcls)
    {
        return -EINVAL;
    }

    dev= (dev_t)((MV_DEV_T*)pDev)->devt;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,0,0)
    *ppOutDevice = device_create((struct class *)(*ppcls), (struct device *)pDeviceParent, dev, NULL, fmt);
#else
    va_start(vargs, fmt);
    *ppOutDevice = device_create_vargs((struct class *)(*ppcls), (struct device *)pDeviceParent, dev, NULL, fmt, vargs);
    va_end(vargs);
#endif
    
    if (NULL == *ppOutDevice)
    {
        return -ENOMEM;
    }
    
    return MV_OK;
}
