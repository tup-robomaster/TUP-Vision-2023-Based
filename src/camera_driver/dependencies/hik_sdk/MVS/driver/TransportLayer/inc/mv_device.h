

#ifndef __MV_DEVICE_H__
#define __MV_DEVICE_H__

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

typedef struct _MV_DEV_T_
{
    unsigned int devt;
}MV_DEV_T;

extern int MV_ClassCreate(void** ppOutClass, const char* name);

extern int MV_DeviceCreate(void** ppOutDevice, void** ppInPutClass, const void* pDeviceParent, const void* pDev, const char *fmt, ...);

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // __MV_DEVICE_H__