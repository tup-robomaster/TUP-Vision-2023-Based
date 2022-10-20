#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include "MvCameraControl.h"

void* g_hHandle = NULL;
bool  g_bConnect = false;
char  g_strSerialNumber[64] = {0};

// 等待用户输入enter键来结束取流或结束程序
// wait for user to input enter to stop grabbing or end the sample program
void PressEnterToExit(void)
{
    int c;
    while ( (c = getchar()) != '\n' && c != EOF );
    fprintf( stderr, "\nPress enter to exit.\n");
    while( getchar() != '\n');
}

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

void __stdcall cbException(unsigned int nMsgType, void* pUser)
{
    printf("Device disconnect!\n");
    g_bConnect = false;
}

static void* WorkThread(void* pUser)
{
    int nRet = MV_OK;

    // ch:获取数据包大小 | en:Get payload size
    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(g_hHandle, "PayloadSize", &stParam);
    if (MV_OK != nRet)
    {
        printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
        return NULL;
    }

    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    unsigned char * pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
    if (NULL == pData)
    {
        return NULL;
    }
    unsigned int nDataSize = stParam.nCurValue;

    while(1)
    {
        if(!g_bConnect)
        {
            break;
        }
        nRet = MV_CC_GetOneFrameTimeout(g_hHandle, pData, nDataSize, &stImageInfo, 1000);
        if (nRet == MV_OK)
        {
            printf("GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n", 
                stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
        }
        else
        {
            printf("no data[%x]\n", nRet);
        }
    }

    free(pData);
    return 0;
}

static void* ReconnectProcess(void* pUser)
{
    int nRet = MV_OK;
    MV_CC_DEVICE_INFO_LIST stDeviceList = {0};

    while(1)
    {
        if (true == g_bConnect)
        {
            sleep(1);
            continue;
        }

        nRet = MV_CC_StopGrabbing(g_hHandle);
        nRet = MV_CC_CloseDevice(g_hHandle);
        nRet = MV_CC_DestroyHandle(g_hHandle);
        g_hHandle = NULL;

        printf("connecting...\n");
        // 枚举设备
        // enum device
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
            continue;
        }

        // 根据序列号选择相机
        // Select camera by serial number 
        unsigned int nIndex = -1;
        if (stDeviceList.nDeviceNum > 0)
        {
            for (int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    continue;
                } 


                if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) 
                {
                    if (!strcmp((char*)(pDeviceInfo->SpecialInfo.stGigEInfo.chSerialNumber), g_strSerialNumber))
                    {
                        nIndex = i;
                        break;
                    }
                }
                else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE)
                {
                    if (!strcmp((char*)(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber), g_strSerialNumber))
                    {
                        nIndex = i;
                        break;
                    }
                }

            }  
        } 
        else
        {
            continue;
        }

        if (-1 == nIndex)
        {
            continue;
        }

        // 选择设备并创建句柄
        // select device and create handle
        nRet = MV_CC_CreateHandle(&g_hHandle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            continue;
        }

        // 打开设备
        // open device
        nRet = MV_CC_OpenDevice(g_hHandle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            continue;
        }

        g_bConnect = true;
		
        // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(g_hHandle);
            if (nPacketSize > 0)
            {
                nRet = MV_CC_SetIntValue(g_hHandle,"GevSCPSPacketSize",nPacketSize);
                if(nRet != MV_OK)
                {
                    printf("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
                }
            }
            else
            {
                printf("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
            }
        }
		
        // 注册异常回调
        // register exception callback
        nRet = MV_CC_RegisterExceptionCallBack(g_hHandle, cbException, NULL);
        if (MV_OK != nRet)
        {
            printf("MV_CC_RegisterExceptionCallBack fail! nRet [%x]\n", nRet);
            continue;
        }
        printf("connect succeed\n");

        // 开始取流
        // start grab image
        nRet = MV_CC_StartGrabbing(g_hHandle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            continue;
        }

        pthread_t nThreadID;
        nRet = pthread_create(&nThreadID, NULL ,WorkThread , NULL);
        if (nRet != 0)
        {
            printf("thread create failed.ret = %d\n",nRet);
            continue;
        }
    }
    return 0;
}

int main()
{
    int nRet = MV_OK;
    MV_CC_DEVICE_INFO_LIST stDeviceList = {0};
    unsigned int nSelectNum = 0;
    // 枚举设备
    // enum device
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
    {
        printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
        return -1;
    }
    if (stDeviceList.nDeviceNum > 0)
    {
        for (int i = 0; i < stDeviceList.nDeviceNum; i++)
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
        return -1;
    }

    printf("Please Intput camera index: ");
    scanf("%d", &nSelectNum);

    if (nSelectNum >= stDeviceList.nDeviceNum)
    {
        printf("Intput error!\n");
        return -1;
    }

    if (stDeviceList.pDeviceInfo[nSelectNum]->nTLayerType == MV_GIGE_DEVICE) 
    {
        memcpy(g_strSerialNumber, stDeviceList.pDeviceInfo[nSelectNum]->SpecialInfo.stGigEInfo.chSerialNumber, 
            sizeof(stDeviceList.pDeviceInfo[nSelectNum]->SpecialInfo.stGigEInfo.chSerialNumber));
    }
    else if (stDeviceList.pDeviceInfo[nSelectNum]->nTLayerType == MV_USB_DEVICE)
    {
        memcpy(g_strSerialNumber, stDeviceList.pDeviceInfo[nSelectNum]->SpecialInfo.stUsb3VInfo.chSerialNumber, 
            sizeof(stDeviceList.pDeviceInfo[nSelectNum]->SpecialInfo.stUsb3VInfo.chSerialNumber));
    }


    pthread_t nThreadID;
    nRet = pthread_create(&nThreadID, NULL, ReconnectProcess, NULL);
    if (nRet != 0)
    {
        printf("thread create failed nRet = %d\n",nRet);
        return -1;
    }

    PressEnterToExit();
    g_bConnect = false;

    // 关闭设备
    // close device
    nRet = MV_CC_CloseDevice(g_hHandle);
    // 销毁句柄
    // destroy handle
    nRet = MV_CC_DestroyHandle(g_hHandle);

    printf("exit\n");
    return 0;
}
