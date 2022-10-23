/***************************************************************************************************
 * @file      SaveImage.java
 * @breif     Use functions provided in MvCameraControlWrapper.jar to decode the lossless compressed data。
 * @author    hulongcheng
 * @date      2021/10/15
 *
 * @warning
 * @version   V1.0.0  2021/10/15 Create this file
 * @since     2021/10/15
 **************************************************************************************************/

import java.io.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Scanner;

import MvCameraControlWrapper.*;
import static MvCameraControlWrapper.MvCameraControl.*;
import static MvCameraControlWrapper.MvCameraControlDefines.*;
public class HighBandwidthDecode {
    private static void printDeviceInfo(MV_CC_DEVICE_INFO stDeviceInfo)
    {
        if (null == stDeviceInfo) {
            System.out.println("stDeviceInfo is null");
            return;
        }

        if (stDeviceInfo.transportLayerType == MV_GIGE_DEVICE) {
            System.out.println("\tCurrentIp:       " + stDeviceInfo.gigEInfo.currentIp);
            System.out.println("\tModel:           " + stDeviceInfo.gigEInfo.modelName);
            System.out.println("\tUserDefinedName: " + stDeviceInfo.gigEInfo.userDefinedName);
        } else if (stDeviceInfo.transportLayerType == MV_USB_DEVICE) {
            System.out.println("\tUserDefinedName: " + stDeviceInfo.usb3VInfo.userDefinedName);
            System.out.println("\tSerial Number:   " + stDeviceInfo.usb3VInfo.serialNumber);
            System.out.println("\tDevice Number:   " + stDeviceInfo.usb3VInfo.deviceNumber);
        } else {
            System.err.print("Device is not supported! \n");
        }

        System.out.println("\tAccessible:      "
                + MvCameraControl.MV_CC_IsDeviceAccessible(stDeviceInfo, MV_ACCESS_Exclusive));
        System.out.println("");
    }

    private static void printFrameInfo(MV_FRAME_OUT_INFO stFrameInfo)
    {
        if (null == stFrameInfo)
        {
            System.err.println("stFrameInfo is null");
            return;
        }

        StringBuilder frameInfo = new StringBuilder("");
        frameInfo.append(("\tFrameNum[" + stFrameInfo.frameNum + "]"));
        frameInfo.append("\tWidth[" + stFrameInfo.width + "]");
        frameInfo.append("\tHeight[" + stFrameInfo.height + "]");
        frameInfo.append("\tframeLen[" + stFrameInfo.frameLen + "]");
        frameInfo.append(String.format("\tPixelType[%#x]", stFrameInfo.pixelType.getnValue()));

        System.out.println(frameInfo.toString());
    }

    public static void saveDataToFile(byte[] dataToSave, int dataSize, String fileName)
    {
        OutputStream os = null;

        try
        {
            // Create directory
            File tempFile = new File("dat");
            if (!tempFile.exists())
            {
                tempFile.mkdirs();
            }

            os = new FileOutputStream(tempFile.getPath() + File.separator + fileName);
            os.write(dataToSave, 0, dataSize);
            System.out.println("SaveImage succeed.");
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
        finally
        {
            // Close file stream
            try
            {
                os.close();
            }
            catch (IOException e)
            {
                e.printStackTrace();
            }
        }
    }

    public static int chooseCamera(ArrayList<MV_CC_DEVICE_INFO> stDeviceList)
    {
        if (null == stDeviceList)
        {
            return -1;
        }

        // Choose a device to operate
        int camIndex = -1;
        Scanner scanner = new Scanner(System.in);

        while (true)
        {
            try
            {
                System.out.print("Please input camera index (-1 to quit):");
                camIndex = scanner.nextInt();
                if ((camIndex >= 0 && camIndex < stDeviceList.size()) || -1 == camIndex)
                {
                    break;
                }
                else
                {
                    System.out.println("Input error: " + camIndex);
                }
            }
            catch (Exception e)
            {
                e.printStackTrace();
                camIndex = -1;
                break;
            }
        }
        scanner.close();

        if (-1 == camIndex)
        {
            System.out.println("Bye.");
            return camIndex;
        }

        if (0 <= camIndex && stDeviceList.size() > camIndex)
        {
            if (MV_GIGE_DEVICE == stDeviceList.get(camIndex).transportLayerType)
            {
                System.out.println("Connect to camera[" + camIndex + "]: " + stDeviceList.get(camIndex).gigEInfo.userDefinedName);
            }
            else if (MV_USB_DEVICE == stDeviceList.get(camIndex).transportLayerType)
            {
                System.out.println("Connect to camera[" + camIndex + "]: " + stDeviceList.get(camIndex).usb3VInfo.userDefinedName);
            }
            else
            {
                System.out.println("Device is not supported.");
            }
        }
        else
        {
            System.out.println("Invalid index " + camIndex);
            camIndex = -1;
        }

        return camIndex;
    }

    public static void main(String[] args)
    {
        int nRet = MV_OK;
        int camIndex = -1;
        Handle hCamera = null;
        ArrayList<MV_CC_DEVICE_INFO> stDeviceList;

        do
        {
            System.out.println("SDK Version " + MvCameraControl.MV_CC_GetSDKVersion());

            // Enuerate GigE and USB devices
            try
            {
                stDeviceList = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE);
                if (0 >= stDeviceList.size())
                {
                    System.out.println("No devices found!");
                    break;
                }
                int i = 0;
                for (MV_CC_DEVICE_INFO stDeviceInfo : stDeviceList)
                {
                    System.out.println("[camera " + (i++) + "]");
                    printDeviceInfo(stDeviceInfo);
                }
            }
            catch (CameraControlException e)
            {
                System.err.println("Enumrate devices failed!" + e.toString());
                e.printStackTrace();
                break;
            }

            // choose camera
            camIndex = chooseCamera(stDeviceList);
            if (camIndex == -1)
            {
                break;
            }

            // Create handle
            try
            {
                hCamera = MvCameraControl.MV_CC_CreateHandle(stDeviceList.get(camIndex));
            }
            catch (CameraControlException e)
            {
                System.err.println("Create handle failed!" + e.toString());
                e.printStackTrace();
                hCamera = null;
                break;
            }

            // Open device
            nRet = MvCameraControl.MV_CC_OpenDevice(hCamera);
            if (MV_OK != nRet)
            {
                System.err.printf("Connect to camera failed, errcode: [%#x]\n", nRet);
                break;
            }

            // Make sure that trigger mode is off
            nRet = MvCameraControl.MV_CC_SetEnumValueByString(hCamera, "TriggerMode", "Off");
            if (MV_OK != nRet)
            {
                System.err.printf("SetTriggerMode failed, errcode: [%#x]\n", nRet);
                break;
            }

            // Get payload size
            MVCC_INTVALUE stParam = new MVCC_INTVALUE();
            nRet = MvCameraControl.MV_CC_GetIntValue(hCamera, "PayloadSize", stParam);
            if (MV_OK != nRet)
            {
                System.err.printf("Get PayloadSize fail, errcode: [%#x]\n", nRet);
                break;
            }

            // Start grabbing
            nRet = MvCameraControl.MV_CC_StartGrabbing(hCamera);
            if (MV_OK != nRet)
            {
                System.err.printf("Start Grabbing fail, errcode: [%#x]\n", nRet);
                break;
            }
            System.err.printf("PayloadSize:[%d]\n", stParam.curValue);
            // Get one frame
            MV_FRAME_OUT_INFO stImageInfo = new MV_FRAME_OUT_INFO();
            byte[] pData = new byte[(int)stParam.curValue];

            int nImageNum = 10;
            for (int i = 0; i < nImageNum; i++) {
                nRet = MvCameraControl.MV_CC_GetOneFrameTimeout(hCamera, pData, stImageInfo, 1000);
                if (MV_OK != nRet)
                {
                    System.err.printf("GetOneFrameTimeout fail, errcode:[%#x]\n", nRet);
                    break;
                }

                if (stImageInfo.lostPacket > 0) {
                    System.err.printf("Lost Pacekt Number %d\n", stImageInfo.lostPacket);
                    continue;
                }

                System.out.println("GetOneFrame: ");
                printFrameInfo(stImageInfo);
                MV_CC_HB_DECODE_PARAM stDecodeParam = new MV_CC_HB_DECODE_PARAM();
                stDecodeParam.pSrcBuf = pData;
                stDecodeParam.nSrcLen = stImageInfo.frameLen;

                byte[] pDstBuf = new byte[(int) stParam.curValue];
                stDecodeParam.pDstBuf = pDstBuf;
                stDecodeParam.nDstBufSize = pDstBuf.length;
                nRet = MvCameraControl.MV_CC_HB_Decode(hCamera, stDecodeParam);
                if (MV_OK != nRet)
                {
                    System.err.printf("MV_CC_HB_Decode fail, errcode:[%#x]\n", nRet);
                    break;
                }

                StringBuffer fileName = new StringBuffer();
                fileName.append("Image_");
                fileName.append(stDecodeParam.nWidth);
                fileName.append("_");
                fileName.append(stDecodeParam.nHeight);
                fileName.append("_");
                fileName.append(stImageInfo.frameNum);
                fileName.append(".raw");
                // Save buffer content to file
                saveDataToFile(pDstBuf, stDecodeParam.nDstBufLen, fileName.toString());

            }

            // Stop grabbing
            nRet = MvCameraControl.MV_CC_StopGrabbing(hCamera);
            if (MV_OK != nRet)
            {
                System.err.printf("StopGrabbing fail, errcode: [%#x]\n", nRet);
                break;
            }
        } while (false);

        if (null != hCamera)
        {
            // Destroy handle
            nRet = MvCameraControl.MV_CC_DestroyHandle(hCamera);
            if (MV_OK != nRet) {
                System.err.printf("DestroyHandle failed, errcode: [%#x]\n", nRet);
            }
        }
    }
}
