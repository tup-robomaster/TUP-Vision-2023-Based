#include "CameraApi.h" // import MindVision API
#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;
using namespace std;

class Camera {
    public:
        int iCameraCounts = 1;
        int hCamera;
        tSdkCameraDevInfo tCameraEnumList;
        tSdkCameraCapbility tCapability;
        tSdkFrameHead sFrameInfo;
        BYTE* pbyBuffer;
        Mat frame;

        // bool usingAe = false;          // Using Auto Exposure (NOTE: IF usingAe is setted to true, then any changes of exposureTimeValue will not be effective)
        // bool usingAutoWb = true;       // Using Auto White Balence
        // int exposureTimeValue = 9999;  // Exposure Time Value Setting
        // int sharpnessValue = 0;        // Sharpness Value Setting
        // int gainRValue = 0;            // Gain Red Channel Value Setting
        // int gainGValue = 0;            // Gain Green Channel Value Setting
        // int gainBValue = 0;            // Gain Blue Channel Value Setting
        // int saturationValue = 50;      // Saturation Value Setting

        int updateParams();              // TODO: ROS update params dynamically
        CameraSdkStatus initCamera();

        // pass the parameters so that the camera will update the parameters dynamically
        int settingParams(bool usingAe, int exposureTimeValue, int sharpnessValue, bool usingAutoWb, int saturationValue);
        int prepareCamera();
        int convertToMat();
        int releaseCamera();

};