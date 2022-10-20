Machine Vision Camera Linux SDK User Manual
===========================================================================
Version: 2.4.0.5

Camera supported : GigE and USB3 Camera

OS supported: ubuntu 14.04 (32/64 bits), ubuntu 16.04 (32/64 bits), 
              centos7 (32/64 bits), redhat (64 bits)
===========================================================================

Runtime Configuration
===========================================================================
Before make any samples,please check the environment variables,LD_LIBRARY_PATH must contain SDK path,
and MVCAM_COMMON_RUNENV is set to "/opt/MVS/lib"; if not, please cd to the install directory which \
contain the setup.sh, and source the set_env_path.sh shell.

===========================================================================

Program demostration
===========================================================================
Display: Image dispaly sample
The Created image window is not support strech.
1. Create image window by using xlib library;
2. Enumerate device, select device and create handle, and then open device;
3. Start grabbing, call dispaly function to input window handle;
4. Input enter to stop grabbing.

===========================================================================
ForceIP: Set forceip
1. Enumerate device, select device and create handle;
2. Set forceip;
3. Input enter to stop grabbing.

===========================================================================
Grab_ImageCallback: Grabbing image by callback
1. Enumerate device, select device and create handle, and then open device;
2. Set trigger mode as off;
3. Start grabbing;
4. If image data is grabbed, ImageCallBackEx will be called;
5. Input enter to stop grabbing.

===========================================================================
GrabImage: Actively grab image
1. Enumerate device, select device and create handle, and then open device;
2. Set trigger mode as off;
3. Start grabbing, open a thread to grab image data;
4. If image data is grabbed, MV_OK will be returned by MV_CC_GetOneFrameTimeout;
5. Input enter to stop grabbing.

===========================================================================
GrabMultipleCamera: Multiple camera grabbing
1. Enumerate device, select device and create handle, and then open device;
2. Set trigger mode as off;
3. Start grabbing, open a thread to grab image data;
4. If image data is grabbed, MV_OK will be returned by MV_CC_GetOneFrameTimeout;
5. Input enter to stop grabbing.

===========================================================================
ImageProcess: Image processing (Save image and pixel format conversion)
1. Enumerate device, select device and create handle, and then open device;
2. Start grabbing, if image data is grabbed, MV_OK will be returned by MV_CC_GetOneFrameTimeout;
3. Select case 0, 1 or 2 to do different image processing;
4. Input enter to stop grabbing.

===========================================================================
ReconnectDemo: Re-connect sample
1. Open a thread to re-connect camera, which contains enumeration, 
   create handle, open device, register exception callback and other functions
2. If there is a camera lost connect by exception, re-enumrate and connect the 0th camera;
3. Input enter to stop grabbing.

===========================================================================
SetIO: Set IO
1. Enumerate device, select device and create handle, and then open device;
2. Get LineSelector, and set LineSelector;
3. Get LineMode, and set LineMode;
4. Input enter to stop grabbing.

===========================================================================
SetParam: Set parameters
1. Enumerate device, select device and create handle, and then open device;
2. Set or get int type parameters;
3. Set or get float type parameters;
4. Set or get enum type parameters;
5. Set or get bool type parameters;
6. Set or get string type parameters;
7. Input enter to stop grabbing.

===========================================================================
Trigger_Image: Grabbing by trigger
1. Enumerate device, select device and create handle, and then open device;
2. Set trigger mode as on, and trigger source as software;
3. Start grabbing, open a thread to send trigger command and get image data;
4. Input enter to stop grabbing.

===========================================================================
Trigger_ImageCallback: Grabbing by trigger in callback
1. Enumerate device, select device and create handle, and then open device;
2. Set trigger mode as on, and trigger source as software;
3. Start grabbing, open a thread to send trigger command;
4. If image data is grabbed, ImageCallBackEx will be called;
5. Input enter to stop grabbing.

===========================================================================
ConnectSpecCamera: Connect camera without enumerating (Remote access)
1. Set camera IP and NIC IP;
2. Create handle and connect camera;
3. Start grabbing.

===========================================================================
Events: Camera event
1. Enumerate device, select device and create handle, and then open device;
2. Enabled Event;
3. Register Event Callback (Optional single, multiple or full events) 
4. Start grabbing, enter callback once camera event happens.

===========================================================================
MultiCast: Multicast grabbing
1. Enumerate device, select device and create handle, and then open device;
2. Set control or monitor mode;
3. Input IP and port of multicast group;
4. Start grabbing (Noted that start grabbing on monitor requires the grabbing on control side).

===========================================================================
ParametrizeCamera_FileAccess: 
1. Enumerate device, select device and create handle, and then open device;
2. Execute a thread, read camera configure file by FileAccess;
3. Execute a thread, write camera configure file into camera by FileAccess.

===========================================================================
ParametrizeCamera_LoadAndSave:
1. Enumerate device, select device and create handle, and then open device;
2. Export camera feature into a file;
3. Import camera feature from above file.


