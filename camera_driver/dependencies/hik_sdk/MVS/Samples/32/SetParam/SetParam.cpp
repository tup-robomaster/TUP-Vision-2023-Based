#include <stdio.h>
#include <string.h>
#include "MvCameraControl.h"

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

int main()
{
    int nRet = MV_OK;

    void* handle = NULL;
    do 
    {
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

        // 枚举设备
        // enum device
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
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

        // 打开设备
        // open device
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            break;
        }

        // 获取int型变量
        // get IInteger variable
        MVCC_INTVALUE stHeight = {0};
        nRet = MV_CC_GetIntValue(handle, "Height", &stHeight);
        if (MV_OK == nRet)
        {
            printf("height current value:%d\n", stHeight.nCurValue);
            printf("height max value:%d\n", stHeight.nMax);
            printf("height min value:%d\n", stHeight.nMin);
            printf("height increment value:%d\n\n", stHeight.nInc);
        }
        else
        {
            printf("get height failed! nRet [%x]\n\n", nRet);
        }

        // 设置int型变量
        // set IInteger variable
        unsigned int nHeightValue = 0;
        printf("please input the height to set:");
        if(0 == scanf("%d", &nHeightValue))
        {
            printf("Input Format Error!");
            break;
        }

        // 宽高设置时需考虑步进(16)，即设置宽高需16的倍数
        // Step (16) should be considered when setting width and height, that is the width and height should be a multiple of 16
        nRet = MV_CC_SetIntValue(handle, "Height", nHeightValue);    
        if (MV_OK == nRet)
        {
            printf("set height OK!\n\n");
        }
        else
        {
            printf("set height failed! nRet [%x]\n\n", nRet);
        }

        // 获取float型变量
        // get IFloat variable
        MVCC_FLOATVALUE stExposureTime = {0};
        nRet = MV_CC_GetFloatValue(handle, "ExposureTime", &stExposureTime);
        if (MV_OK == nRet)
        {
            printf("exposure time current value:%f\n", stExposureTime.fCurValue);
            printf("exposure time max value:%f\n", stExposureTime.fMax);
            printf("exposure time min value:%f\n\n", stExposureTime.fMin);
        }
        else
        {
            printf("get exposure time failed! nRet [%x]\n\n", nRet);
        }

        // 设置float型变量
        // set IFloat variable
        float fExposureTime = 0.0f;
        printf("please input the exposure time to set: ");
        if(0 == scanf("%f", &fExposureTime))
        {
            printf("Input Format Error!");
            break;
        }

        nRet = MV_CC_SetFloatValue(handle, "ExposureTime", fExposureTime);
        if (MV_OK == nRet)
        {
            printf("set exposure time OK!\n\n");
        }
        else
        {
            printf("set exposure time failed! nRet [%x]\n\n", nRet);
        }

        // 获取enum型变量
        // get IEnumeration variable
        MVCC_ENUMVALUE stTriggerMode = {0};
        nRet = MV_CC_GetEnumValue(handle, "TriggerMode", &stTriggerMode);
        if (MV_OK == nRet)
        {
            printf("TriggerMode current value:%d\n", stTriggerMode.nCurValue);

            printf("supported TriggerMode number:%d\n", stTriggerMode.nSupportedNum);

            for (unsigned int i = 0; i < stTriggerMode.nSupportedNum; ++i)
            {
                printf("supported TriggerMode [%d]:%d\n", i, stTriggerMode.nSupportValue[i]);
            }
            printf("\n");
        }
        else
        {
            printf("get TriggerMode failed! nRet [%x]\n\n", nRet);
        }

        // 设置enum型变量
        // set IEnumeration variable
        unsigned int nTriggerMode = 0;
        printf("please input the TriggerMode to set:");
        if(0 == scanf("%d", &nTriggerMode))
        {
            printf("Input Format Error!");
            break;
        }

        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", nTriggerMode);
        if (MV_OK == nRet)
        {
            printf("set TriggerMode OK!\n\n");
        }
        else
        {
            printf("set TriggerMode failed! nRet [%x]\n\n", nRet);
        }

        // 获取bool型变量
        // get IBoolean variable
        bool bGetBoolValue = false;
        nRet = MV_CC_GetBoolValue(handle, "ReverseX", &bGetBoolValue);
        if (MV_OK == nRet)
        {
            if (0 != bGetBoolValue)
            {
                printf("ReverseX current is true\n\n");
            }
            else
            {
                printf("ReverseX current is false\n\n");
            }
        }
        else
        {
            printf("get ReverseX Failed! nRet = [%x]\n\n", nRet);
        }

        // 设置bool型变量
        // set IBoolean variable
        int nSetBoolValue;
        bool bSetBoolValue;
        printf("please input the ReverseX to set(0 or 1): ");
        if(0 == scanf("%d", &nSetBoolValue))
        {
            printf("Input Format Error!");
            break;
        }

        if (0 != nSetBoolValue)
        {
            bSetBoolValue = true;
        }
        else
        {
            bSetBoolValue = false;
        }
        nRet = MV_CC_SetBoolValue(handle, "ReverseX", bSetBoolValue);
        if (MV_OK == nRet)
        {
            printf("Set ReverseX OK!\n\n");
        }
        else
        {
            printf("Set ReverseX Failed! nRet = [%x]\n\n", nRet);
        }

        // 获取string型变量
        // get IString variable
        MVCC_STRINGVALUE stStringValue = {0};
        nRet = MV_CC_GetStringValue(handle, "DeviceUserID", &stStringValue);
        if (MV_OK == nRet)
        {
            printf("Get DeviceUserID [%s]\n\n", stStringValue.chCurValue);
        }
        else
        {
            printf("Get DeviceUserID Failed! nRet = [%x]\n\n", nRet);
        }

        // 设置string型变量
        // set IString variable
        unsigned char strValue[256];
        printf("please input the DeviceUserID to set(string):");
        if(0 == scanf("%s", strValue))
        {
            printf("Input Format Error!");
            break;
        }

        nRet = MV_CC_SetStringValue(handle, "DeviceUserID", (char*)strValue);
        if (MV_OK == nRet)
        {
            printf("Set DeviceUserID OK!\n\n");
        }
        else
        {
            printf("Set DeviceUserID Failed! nRet = [%x]\n\n", nRet);
        }

        // 关闭设备
        // close device
        nRet = MV_CC_CloseDevice(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
            break;
        }

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
