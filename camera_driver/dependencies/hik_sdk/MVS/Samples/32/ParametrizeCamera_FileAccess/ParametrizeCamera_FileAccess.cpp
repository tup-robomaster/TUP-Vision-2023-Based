#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include "MvCameraControl.h"

unsigned int g_nMode = 0;
int g_nRet = MV_OK;

bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }

    return true;
}

static void*  ProgressThread(void* pUser)
{
    int nRet = MV_OK;
    MV_CC_FILE_ACCESS_PROGRESS stFileAccessProgress = {0};

    while(1)
    {
        //ch:获取文件存取进度 |en:Get progress of file access
        nRet = MV_CC_GetFileAccessProgress(pUser, &stFileAccessProgress);
        if(4 == sizeof(long))
        {
            printf("State = 0x%x,Completed = %lld,Total = %lld\r\n", 
                nRet, stFileAccessProgress.nCompleted, stFileAccessProgress.nTotal);
        }
        else
        {
            printf("State = 0x%x,Completed = %ld,Total = %ld\r\n", 
                nRet, stFileAccessProgress.nCompleted, stFileAccessProgress.nTotal);
        }
        
        if (nRet != MV_OK || (stFileAccessProgress.nCompleted != 0 && stFileAccessProgress.nCompleted == stFileAccessProgress.nTotal))
        {
            break;
        }

        usleep(50000);
    }

    return 0;
}

static void*  FileAccessThread(void* pUser)
{
    MV_CC_FILE_ACCESS stFileAccess = {0};

    stFileAccess.pUserFileName = "UserSet1.bin";
    stFileAccess.pDevFileName = "UserSet1";
    if (1 == g_nMode)
    {
        //ch:读模式 |en:Read mode
        g_nRet = MV_CC_FileAccessRead(pUser, &stFileAccess);
        if (MV_OK != g_nRet)
        {
            printf("File Access Read fail! nRet [0x%x]\n", g_nRet);
        }
    }
    else if (2 == g_nMode)
    {
        //ch:写模式 |en:Write mode
        g_nRet = MV_CC_FileAccessWrite(pUser, &stFileAccess);
        if (MV_OK != g_nRet)
        {
            printf("File Access Write fail! nRet [0x%x]\n", g_nRet);
        }
    }

    return 0;
}

int main()
{
    int nRet = MV_OK;
    void* handle = NULL;

    do 
    {
        // ch:枚举设备 | en:Enum device
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("Enum Devices fail! nRet [0x%x]\n", nRet);
            break;
        }

        if (stDeviceList.nDeviceNum > 0)
        {
            for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                } 
                PrintDeviceInfo(pDeviceInfo);            
            }  
        } 
        else
        {
            printf("Find No Devices!\n");
            break;
        }

        printf("Please Intput camera index: ");
        unsigned int nIndex = 0;
        scanf("%d", &nIndex);

        if (nIndex >= stDeviceList.nDeviceNum)
        {
            printf("Intput error!\n");
            break;
        }

        // ch:选择设备并创建句柄 | en:Select device and create handle
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            printf("Create Handle fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:打开设备 | en:Open device
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            printf("Open Device fail! nRet [0x%x]\n", nRet);
            break;
        }

        //ch:读模式 |en:Read mode
        g_nMode = 1;
        printf("Read to file.\n");

        pthread_t nReadHandle;
        nRet = pthread_create(&nReadHandle, NULL ,FileAccessThread , handle);
        if (nRet != 0)
        {
            break;
        }

        usleep(5000);

        pthread_t nReadProcessHandle; 
        nRet = pthread_create(&nReadProcessHandle, NULL ,ProgressThread , handle);
        if (nRet != 0)
        {
            break;
        }

        void *statusRead;
        void *statusReadProcess;
        pthread_join(nReadHandle, &statusRead);  
        pthread_join(nReadProcessHandle, &statusReadProcess); 
        if (MV_OK == g_nRet)
        {
            printf("File Access Read Success!\n");
        }
        printf("\n");

        //ch:写模式 |en:Write mode
        g_nMode = 2;
        printf("Write from file.\n");

        pthread_t nWriteHandle; 
        nRet = pthread_create(&nWriteHandle, NULL ,FileAccessThread , handle);
        if (nRet != 0)
        {
            break;
        }

        usleep(5000);

        pthread_t nWriteProgressHandle; 
        nRet = pthread_create(&nWriteProgressHandle, NULL ,ProgressThread , handle);
        if (nRet != 0)
        {
            break;
        }

        void *statusWrite;
        void *statusWriteProcess;
        pthread_join(nWriteHandle, &statusWrite);  
        pthread_join(nWriteProgressHandle, &statusWriteProcess); 
        if (MV_OK == g_nRet)
        {
            printf("File Access Write Success!\n");
        }

        // ch:关闭设备 | Close device
        nRet = MV_CC_CloseDevice(handle);
        if (MV_OK != nRet)
        {
            printf("ClosDevice fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:销毁句柄 | Destroy handle
        nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet)
        {
            printf("Destroy Handle fail! nRet [0x%x]\n", nRet);
            break;
        }
    } while (0);

    if (nRet != MV_OK)
    {
        if (handle != NULL)
        {
            MV_CC_DestroyHandle(handle);
            handle = NULL;
        }
    }

    printf("exit.\n");

    return 0;
}
