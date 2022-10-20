#include "DxImageProc.h"
#include "GxIAPI.h"

#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
    // 初始化
    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXInitLib();
    if (status != GX_STATUS_SUCCESS)
    {
        cout << "GXInitLib failed" << endl;
        return -1;
    }
    cout << "Yes" << endl;
    return 0;
}
