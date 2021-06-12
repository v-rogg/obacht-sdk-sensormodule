#include <iostream>
#include "vendor/sdk/include/rplidar.h"
#include <csignal>
#include <mqtt/async_client.h>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace rp::standalone::rplidar;

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

const std::string MQTT_SERVER_ADDRESS { "tcp://192.168.178.48:1883" };
const std::string TOPIC { "test" };
const int QOS = 1;

int main() {
    u_result op_result;

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


        drv->startMotor();
        drv->startScan(0,0);

        printf("-----------\n");
        printf("Start scan\n");
        printf("-----------\n");

        signal(SIGINT, ctrlc);

        while(true) {

            rplidar_response_measurement_node_hq_t nodes[8192];
            size_t count = _countof(nodes);

            op_result = drv->grabScanDataHq(nodes, count);
            if (IS_OK(op_result)) {
                drv->ascendScanData(nodes, count);
                for (int pos = 0; pos < (int)count; pos++) {
                    printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
                        (nodes[pos].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ",
                        (nodes[pos].angle_z_q14 * 90.f / (1 << 14)),
                        nodes[pos].dist_mm_q2/4.0f,
                        nodes[pos].quality);
                }
            }

            if (ctrl_c_pressed) {
                break;
            }
        }

        drv->stop();
        drv->stopMotor();
        drv->disconnect();

    } else {
        fprintf(stderr, "Failed to connect to LIDAR %08x\r\n", res);
    }

    on_finished:
    RPlidarDriver::DisposeDriver(drv);

    return 0;
}
