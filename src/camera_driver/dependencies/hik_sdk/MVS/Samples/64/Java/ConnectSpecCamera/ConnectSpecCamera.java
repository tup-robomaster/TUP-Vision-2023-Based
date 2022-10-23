/***************************************************************************************************
 * @file      ConnectSpecCamera.java
 * @breif     Use functions provided in MvCameraControlWrapper.jar to grab images
 * @author    hulongcheng
 * @date      2021/07/12
 *
 * @warning
 * @version   V1.0.0  2021/07/12 Create this file
 * @since     2021/07/14
 **************************************************************************************************/


import MvCameraControlWrapper.CameraControlException;
import MvCameraControlWrapper.MvCameraControl;
import MvCameraControlWrapper.MvCameraControlDefines;

import java.util.ArrayList;
import java.util.NoSuchElementException;
import java.util.Scanner;

import static MvCameraControlWrapper.MvCameraControl.*;
import static MvCameraControlWrapper.MvCameraControlDefines.*;

public class ConnectSpecCamera {


    private static void printDeviceInfo(MV_CC_DEVICE_INFO stDeviceInfo) {
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

    private static void printFrameInfo(MV_FRAME_OUT_INFO stFrameInfo) {
        if (null == stFrameInfo) {
            System.err.println("stFrameInfo is null");
            return;
        }

        StringBuilder frameInfo = new StringBuilder("");
        frameInfo.append(("\tFrameNum[" + stFrameInfo.frameNum + "]"));
        frameInfo.append("\tWidth[" + stFrameInfo.width + "]");
        frameInfo.append("\tHeight[" + stFrameInfo.height + "]");
        frameInfo.append(String.format("\tPixelType[%#x]", stFrameInfo.pixelType.getnValue()));

        System.out.println(frameInfo.toString());
    }


    private static int chooseCamera(ArrayList<MV_CC_DEVICE_INFO> stDeviceList, Scanner scanner) {
        if (null == stDeviceList) {
            return -1;
        }

        if (scanner == null) {
            return -1;
        }

        // Choose a device to operate
        int camIndex = -1;

        while (true) {
            try {
                System.out.print("Please input camera index (-1 to quit):");
                camIndex = scanner.nextInt();
                if ((camIndex >= 0 && camIndex < stDeviceList.size()) || -1 == camIndex) {
                    break;
                }
                else {
                    System.out.println("Input error: " + camIndex);
                }
            }
            catch (Exception e) {
                e.printStackTrace();
                camIndex = -1;
                break;
            }
        }

        if (-1 == camIndex) {
            System.out.println("Bye.");
            return camIndex;
        }

        if (stDeviceList.size() > camIndex) {
            if (MV_GIGE_DEVICE == stDeviceList.get(camIndex).transportLayerType) {
                System.out.println("Connect to camera[" + camIndex + "]: " + stDeviceList.get(camIndex).gigEInfo.userDefinedName);
            }
            else if (MV_USB_DEVICE == stDeviceList.get(camIndex).transportLayerType) {
                System.out.println("Connect to camera[" + camIndex + "]: " + stDeviceList.get(camIndex).usb3VInfo.userDefinedName);
            }
            else {
                System.out.println("Device is not supported.");
            }
        }
        else {
            System.out.println("Invalid index " + camIndex);
            camIndex = -1;
        }

        return camIndex;
    }

    static class GetOneFrameThread extends Thread {

        private Handle mHandle;
        private boolean flag = true;

        public GetOneFrameThread(Handle handle) {
            this.mHandle = handle;
        }

        public void stopThread() {
            flag = false;
        }

        @Override
        public void run() {
            super.run();

            // Get payload size
            MVCC_INTVALUE stParam = new MVCC_INTVALUE();
            int nRet = MvCameraControl.MV_CC_GetIntValue(mHandle, "PayloadSize", stParam);
            if (MV_OK != nRet) {
                System.err.printf("Get PayloadSize fail, errcode: [%#x]\n", nRet);
                return;
            }

            while (flag) {
                MV_FRAME_OUT_INFO stImageInfo = new MV_FRAME_OUT_INFO();
                byte[] pData = new byte[(int)stParam.curValue];
                nRet = MvCameraControl.MV_CC_GetOneFrameTimeout(mHandle, pData, stImageInfo, 1000);
                if (MV_OK != nRet) {
                    System.err.printf("GetOneFrameTimeout fail, errcode:[%#x]\n", nRet);
                    break;
                } else {
                    printFrameInfo(stImageInfo);
                }
            }
        }
    }

    public static void main(String[] args) {
        int nRet = MV_OK;
        int camIndex = -1;
        Handle hCamera = null;
        ArrayList<MV_CC_DEVICE_INFO> stDeviceList;
        Scanner scanner = new Scanner(System.in);

        do {
            System.out.println("SDK Version " + MvCameraControl.MV_CC_GetSDKVersion());

            // Enuerate GigE and USB devices
            try {
                stDeviceList = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE);
                if (0 >= stDeviceList.size()) {
                    System.out.println("No devices found!");
                    break;
                }
                int i = 0;
                for (MV_CC_DEVICE_INFO stDeviceInfo : stDeviceList) {
                    System.out.println("[camera " + (i++) + "]");
                    printDeviceInfo(stDeviceInfo);
                }
            }
            catch (CameraControlException e) {
                System.err.println("Enumrate devices failed!" + e.toString());
                e.printStackTrace();
                break;
            }

            // choose camera
            camIndex = chooseCamera(stDeviceList, scanner);
            if (camIndex == -1) {
                break;
            }

            // Create handle
            try {
                hCamera = MvCameraControl.MV_CC_CreateHandle(stDeviceList.get(camIndex));
            }
            catch (CameraControlException e) {
                System.err.println("Create handle failed!" + e.toString());
                e.printStackTrace();
                hCamera = null;
                break;
            }

            // Open device
            nRet = MvCameraControl.MV_CC_OpenDevice(hCamera);
            if (MV_OK != nRet) {
                System.err.printf("Connect to camera failed, errcode: [%#x]\n", nRet);
                break;
            }

            // Make sure that trigger mode is off
            nRet = MvCameraControl.MV_CC_SetEnumValueByString(hCamera, "TriggerMode", "Off");
            if (MV_OK != nRet) {
                System.err.printf("SetTriggerMode failed, errcode: [%#x]\n", nRet);
                break;
            }

            // Start grabbing
            nRet = MvCameraControl.MV_CC_StartGrabbing(hCamera);
            if (MV_OK != nRet) {
                System.err.printf("Start Grabbing fail, errcode: [%#x]\n", nRet);
                break;
            }

            // Get frame
            GetOneFrameThread getOneFrameThread = new GetOneFrameThread(hCamera);
            getOneFrameThread.start();

            // Press enter and enter any characters to end the get frame thread
            while (scanner.hasNext()) {
                getOneFrameThread.stopThread();
                try {
                    getOneFrameThread.join();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                getOneFrameThread = null;
                break;
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

        scanner.close();
    }

}
