#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include "MvCameraControl.h"

// 等待用户输入enter键来结束取流或结束程序
// wait for user to input enter to stop grabbing or end the sample program
void PressEnterToExit(void)
{
    int c;
    fprintf( stderr, "\nPress enter to exit.\n");
    while( getchar() != '\n');
}

bool ConvertToHexIp(unsigned int *nHexIP, unsigned int *nDecIP, char c)
{
    if ( nDecIP[0] < 0 || nDecIP[0] > 255
        || nDecIP[1] < 0 || nDecIP[1] > 255
        || nDecIP[2] < 0 || nDecIP[2] > 255
        || nDecIP[3] < 0 || nDecIP[3] > 255
        || c != '\n')
    {
        return false;
    }
    *nHexIP = (nDecIP[0] << 24) + (nDecIP[1] << 16) + (nDecIP[2] << 8) + nDecIP[3];

    return true;
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

int main()
{
    int nRet = MV_OK;
    void* handle = NULL;
    unsigned int nIP[4] = {0};
    char c = '\0';
    unsigned int nIpAddr = 0, nNetWorkMask = 0, nDefaultGateway = 0;
    do 
    {
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

        // 枚举设备
        // enum device
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
            break;
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

        // 选择设备并创建句柄
        // select device and create handle
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            break;
        }

        // 输入IP 子网掩码 默认网关
        // input ip, subnet mask and defaultway
        printf("Please input ip, example: 192.168.1.100\n");
        int ch;
        if ( 5 != scanf("%d.%d.%d.%d%c", &nIP[0], &nIP[1], &nIP[2], &nIP[3], &c) )
        {
            printf("input count error\n");
            MV_CC_DestroyHandle(handle);
            break;
        }
        if (!ConvertToHexIp(&nIpAddr, nIP, c))
        {
            printf("input IpAddr format is not correct\n");
            MV_CC_DestroyHandle(handle);
            break;
        }

        printf("Please input NetMask, example: 255.255.255.0\n");
        if ( 5 != scanf("%d.%d.%d.%d%c", &nIP[0], &nIP[1], &nIP[2], &nIP[3], &c) )
        {
            printf("input count error\n");
            MV_CC_DestroyHandle(handle);
            break;
        }
        if (!ConvertToHexIp(&nNetWorkMask, nIP, c))
        {
            printf("input NetMask format is not correct\n");
            MV_CC_DestroyHandle(handle);
            break;
        }

        printf("Please input DefaultWay, example: 192.168.1.1\n");
        if ( 5 != scanf("%d.%d.%d.%d%c", &nIP[0], &nIP[1], &nIP[2], &nIP[3], &c) )
        {
            printf("input count error\n");
            MV_CC_DestroyHandle(handle);
            break;
        }
        if (!ConvertToHexIp(&nDefaultGateway, nIP, c))
        {
            printf("input DefaultWay format is not correct\n");
            MV_CC_DestroyHandle(handle);
            break;
        }
		
		//判断设备Ip是否可达
		bool bAccessible = MV_CC_IsDeviceAccessible(stDeviceList.pDeviceInfo[nIndex], MV_ACCESS_Exclusive);
		if(bAccessible)
		{
			// set ipconfig
			nRet = MV_GIGE_SetIpConfig(handle, MV_IP_CFG_STATIC);
			if (MV_OK != nRet)
			{
				printf("MV_GIGE_SetIpConfig fail! nRet [%x]\n", nRet);
				break;
			}
			printf("set IPConfig succeed\n");
			
			// set forceip
			nRet = MV_GIGE_ForceIpEx(handle, nIpAddr, nNetWorkMask, nDefaultGateway);
			if (MV_OK != nRet)
			{	
				printf("MV_GIGE_ForceIpEx fail! nRet [%x]\n", nRet);
				break;
			}
			printf("set IP succeed\n");
		}
		else
		{
			// set forceip
			nRet = MV_GIGE_ForceIpEx(handle, nIpAddr, nNetWorkMask, nDefaultGateway);
			if (MV_OK != nRet)
			{	
				printf("MV_GIGE_ForceIpEx fail! nRet [%x]\n", nRet);
				break;
			}
			printf("set IP succeed\n");
			
			MV_CC_DestroyHandle(handle);
			handle = NULL;
			
			//ch:需要重新创建句柄，设置为静态IP方式进行保存
			stDeviceList.pDeviceInfo[nIndex]->SpecialInfo.stGigEInfo.nCurrentIp = nIpAddr;
			stDeviceList.pDeviceInfo[nIndex]->SpecialInfo.stGigEInfo.nCurrentSubNetMask = nNetWorkMask;
			stDeviceList.pDeviceInfo[nIndex]->SpecialInfo.stGigEInfo.nDefultGateWay = nDefaultGateway;
			nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
			if (MV_OK != nRet)
			{
				printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
				break;
			}
			// set ipconfig
			nRet = MV_GIGE_SetIpConfig(handle, MV_IP_CFG_STATIC);
			if (MV_OK != nRet)
			{
				printf("MV_GIGE_SetIpConfig fail! nRet [%x]\n", nRet);
				break;
			}
			printf("set IPConfig succeed\n");
		}

        PressEnterToExit();

        // 销毁句柄
        // destroy handle
        nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
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
