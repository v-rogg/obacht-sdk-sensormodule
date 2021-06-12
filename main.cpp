#include <iostream>
#include "vendor/sdk/include/rplidar.h"

using namespace rp::standalone::rplidar;

int main() {
    printf("Obacht! Lidar SDK.\n"
           "Version: " RPLIDAR_SDK_VERSION "\n");

    RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if(!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    rplidar_response_device_info_t devinfo;
    std::vector<RplidarScanMode> scanmodes;
    _u16 typicalscanmode;

    #if defined(__GNUC__)
    u_result res = drv->connect("/dev/ttyUSB0", 256000);
    #else
//    u_result res = drv->connect("COM6", 256000);
    u_result res = drv->connect("COM6", 115200);
    #endif

    if(IS_OK(res)) {

        drv->getDeviceInfo(devinfo);
        drv->getAllSupportedScanModes(scanmodes);
        drv->getTypicalScanMode(typicalscanmode);

        drv->disconnect();
    } else {
        fprintf(stderr, "Failed to connect to LIDAR %08x\r\n", res);
    }

    RPlidarDriver::DisposeDriver(drv);

    printf("\n"
           "Firmware Ver: %d.%02d\n"
           "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);

    printf("-----------\n");
    printf("Scan Modes\n");
    printf("-----------\n");

    for(auto &mode : scanmodes) {
        std::cout << mode.scan_mode << std::endl;
    }

    std::cout << typicalscanmode << std::endl;

    return 0;
}
