#include "../../include/armor_detector/inference.h"

int main()
{
    armor_detector::ArmorDetector armor_detect;
    cout << "1" << endl;
    armor_detect.initModel("/home/tup/Desktop/tup_2023/src/vehicle_system/autoaim/armor_detector/model/opt-0527-001.xml");
    cout << "2" << endl;
    return 0;
}