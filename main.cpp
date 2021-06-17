#include <iostream>
#include "vendor/sdk/include/rplidar.h"
#include <csignal>
#include <mqtt/async_client.h>
#include <climits>
#include <unistd.h>
#include <sstream>
//#include <iomanip>
//#include <iostream>
#include <cstddef>
//#include <ctime>
#include <thread>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define NUM_THREADS 3

using namespace rp::standalone::rplidar;

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

const std::string MQTT_SERVER_ADDRESS { "tcp://192.168.178.48:1883" };
const std::string TOPIC { "test" };
const int QOS = 1;

void deviceInfo(RPlidarDriver *drv) {
    rplidar_response_device_info_t devinfo;
    std::vector<RplidarScanMode> scanmodes;
    _u16 typicalscanmode;

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
        std::cout << mode.scan_mode << "  >  " << mode.us_per_sample << "μs per sample  >  " << mode.max_distance << "m" << std::endl;
    }

    std::cout << typicalscanmode << std::endl;
}

void scan(RPlidarDriver *drv, mqtt::topic topic) {
    u_result op_result;

    drv->startMotor();
//            drv->setMotorPWM(700);
    drv->startScan(false, true);

    printf("-----------\n");
    printf("Start scan\n");
    printf("-----------\n");

    while(true) {

        rplidar_response_measurement_node_hq_t nodes[8192];
        size_t count = _countof(nodes);

        op_result = drv->grabScanDataHq(nodes, count);

        if (IS_OK(op_result)) {
            std::stringstream stream;

            timeval curTime{};
            gettimeofday(&curTime, nullptr);
            int milli = curTime.tv_usec / 1000;
            char buffer [80];
            strftime(buffer, 80, "%Y-%m-%dT%H:%M:%S", localtime(&curTime.tv_sec));
            char currentTime[84] = "";
            sprintf(currentTime, "%s.%03d", buffer, milli);
            char timezoneBuffer [80];
            strftime(timezoneBuffer, 80, "%z", localtime(&curTime.tv_sec));

//                    uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();

            stream << "@scan;" << currentTime << timezoneBuffer;
            std::string s = stream.str();
//                    std::cout << s << std::endl;
            topic.publish(s);
            stream.str("");
            stream.clear();

            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count; pos++) {
//                        printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
//                               (nodes[pos].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ",
//                               (nodes[pos].angle_z_q14 * 90.f / (1 << 14)),
//                               nodes[pos].dist_mm_q2/4.0f,
//                               nodes[pos].quality);

                // TODO: Add base 92 converter

                float angle = (nodes[pos].angle_z_q14 * 90.f / (1 << 14));
                float distance = nodes[pos].dist_mm_q2;

                if (distance > 0) stream << std::fixed << angle << ";" << distance << "!";
            }
            s = stream.str();
            topic.publish(s);

//                    drv->stop();
        }

        if (ctrl_c_pressed) {
            break;
        }
    }

    drv->stop();
    drv->stopMotor();
    drv->disconnect();
}

int main() {
    printf("Obacht! Tracking SDK.\n"
           "LiDAR SDK Version: " RPLIDAR_SDK_VERSION "\n");

    char hostname[HOST_NAME_MAX];
    gethostname(hostname, HOST_NAME_MAX);

    mqtt::async_client client(MQTT_SERVER_ADDRESS, hostname);
    RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if(!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    signal(SIGINT, ctrlc);

    try {
        client.connect()->wait();
        mqtt::topic topic(client, TOPIC, QOS);

        u_result res = drv->connect("/dev/ttyUSB0", 256000);
//        u_result res = drv->connect("/dev/ttyUSB0", 115200);

        if(IS_OK(res)) {

            deviceInfo(drv);
//            scan(drv, topic);
            std::thread thScan(scan, drv, topic);
            thScan.join();

        } else {
            fprintf(stderr, "Failed to connect to LIDAR %08x\r\n", res);
        }
    }
    catch (const mqtt::exception& exc) {
        std::cerr << exc << std::endl;
        return 1;
    }

    on_finished:
    RPlidarDriver::DisposeDriver(drv);

    return 0;
}
