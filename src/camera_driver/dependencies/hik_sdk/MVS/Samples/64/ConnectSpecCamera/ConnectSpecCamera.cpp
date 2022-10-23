#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include "MvCameraControl.h"

bool g_bExit = false;
unsigned int g_nPayloadSize = 0;

// 等待用户输入enter键来结束取流或结束程序
// wait for user to input enter to stop grabbing or end the sample program
void PressEnterToExit(void)
{
    int c;
    while ( (c = getchar()) != '\n' && c != EOF );
    fprintf( stderr, "\nPress enter to exit.\n");
    while( getchar() != '\n');
    g_bExit = true;
    sleep(1);
}

static void* WorkThread(void* pUser)
{
    int nRet = MV_OK;

    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    unsigned char * pData = (unsigned char *)malloc(sizeof(unsigned char) * (g_nPayloadSize));
    if (NULL == pData)
    {
        return NULL;
    }
    unsigned int nDataSize = g_nPayloadSize;

    while(1)
    {
		if(g_bExit)
        {
            break;
        }

        nRet = MV_CC_GetOneFrameTimeout(pUser, pData, nDataSize, &stImageInfo, 1000);
        if (nRet == MV_OK)
        {
            printf("Get One Frame: Width[%d], Height[%d], nFrameNum[%d]\n", 
                stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
        }
        else
        {
            printf("No data[0x%x]\n", nRet);
			break;        
		}
	}

    free(pData);
    return 0;
}

int main()
{
    int nRet = MV_OK;
    void* handle = NULL;
    MV_CC_DEVICE_INFO stDevInfo = {0};
    MV_GIGE_DEVICE_INFO stGigEDev = {0};

    // ch:需要连接的相机ip(根据实际填充) | en:The camera IP that needs to be connected (based on actual padding)
    printf("Please input Current Camera Ip : ");
    char nCurrentIp[128];
    scanf("%s", (char*)&nCurrentIp);
    // ch:相机对应的网卡ip(根据实际填充) | en:The pc IP that needs to be connected (based on actual padding)
    printf("Please input Net Export Ip : ");
    char nNetExport[128];
    scanf("%s", (char*)&nNetExport);
    unsigned int nIp1, nIp2, nIp3, nIp4, nIp;

    sscanf(nCurrentIp, "%d.%d.%d.%d", &nIp1, &nIp2, &nIp3, &nIp4);
    nIp = (nIp1 << 24) | (nIp2 << 16) | (nIp3 << 8) | nIp4;
    stGigEDev.nCurrentIp = nIp;

    sscanf(nNetExport, "%d.%d.%d.%d", &nIp1, &nIp2, &nIp3, &nIp4);
    nIp = (nIp1 << 24) | (nIp2 << 16) | (nIp3 << 8) | nIp4;
    stGigEDev.nNetExport = nIp;

    stDevInfo.nTLayerType = MV_GIGE_DEVICE;// ch:仅支持GigE相机 | en:Only support GigE camera
    stDevInfo.SpecialInfo.stGigEInfo = stGigEDev;

    do 
    {
        // ch:选择设备并创建句柄 | en:Select device and create handle
        nRet = MV_CC_CreateHandle(&handle, &stDevInfo);
        if (MV_OK != nRet)
        {
            printf("Create Handle fail! nRet[0x%x]\n", nRet);
            break;
        }

        // ch:打开设备 | en:Open device
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            printf("Open Device fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
		int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
		if (nPacketSize > 0)
		{
			nRet = MV_CC_SetIntValue(handle,"GevSCPSPacketSize",nPacketSize);
			if(nRet != MV_OK)
			{
				printf("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
			}
		}
		else
		{
			printf("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
		}

		
        // ch:设置触发模式为off | en:Set trigger mode as off
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", MV_TRIGGER_MODE_OFF);
        if (MV_OK != nRet)
        {
            printf("Set Trigger Mode fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:获取数据包大小 | en:Get payload size
        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
        if (MV_OK != nRet)
        {
            printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
            break;
        }
        g_nPayloadSize = stParam.nCurValue;

        // ch:开始取流 | en:Start grab image
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
            break;
        }

        pthread_t nThreadID;
        nRet = pthread_create(&nThreadID, NULL ,WorkThread , handle);
        if (nRet != 0)
        {
            printf("thread create failed.ret = %d\n",nRet);
            break;
        }

        PressEnterToExit();  

        // ch:停止取流 | en:Stop grab image
        nRet = MV_CC_StopGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("Stop Grabbing fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:关闭设备 | en:Close device
        nRet = MV_CC_CloseDevice(handle);
        if (MV_OK != nRet)
        {
            printf("Close Device fail! nRet [0x%x]\n", nRet);
            break;
        }

        // ch:销毁句柄 | en:Destroy handle
        nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet)
        {
            printf("Destroy Handle fail! nRet [0x%x]\n", nRet);
            break;
        }
        handle = NULL;
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
