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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

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
//const std::string TOPIC { "test" };
const int QOS = 1;

char hostname[HOST_NAME_MAX];
char *address;
struct hostent *host_entry;


rplidar_response_device_info_t getDeviceInfo(RPlidarDriver *drv) {
    rplidar_response_device_info_t devinfo;
    std::vector<RplidarScanMode> scanmodes;
    _u16 typicalscanmode;

    drv->getDeviceInfo(devinfo);
    drv->getAllSupportedScanModes(scanmodes);
    drv->getTypicalScanMode(typicalscanmode);

    printf("\n"
           "Firmware Ver: %d.%02d\n"
           "Hardware Rev: %d\n"
           "Model: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version
            , devinfo.model);

    printf("-----------\n");
    printf("Scan Modes\n");
    printf("-----------\n");

    for(auto &mode : scanmodes) {
        std::cout << mode.scan_mode << "  >  " << mode.us_per_sample << "μs per sample  >  " << mode.max_distance << "m" << std::endl;
    }

    std::cout << typicalscanmode << std::endl;
    return devinfo;
}

void scan(RPlidarDriver *drv, mqtt::topic topic, mqtt::topic connectionTopic) {
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
            std::stringstream stream;
            stream << "-connect:" << hostname << ":" << address;
            connectionTopic.publish(stream.str());
            std::cout << "Pressed C" << std::endl;
            break;
        }
    }

    drv->stop();
    drv->stopMotor();
    drv->disconnect();
}

void subscription(mqtt::async_client &client) {

    mqtt::topic topic(client, "pingcheck", QOS);

    while (true) {
        auto msg = client.consume_message();
        if (!msg) break;
        if (msg->get_topic() != "test") {
            std::cout << msg->get_topic() << ": " << msg->to_string() << std::endl;
            topic.publish("@scan:2006-01-02T15:04:05Z07:00");
        }
    }
}

int main() {
    printf("Obacht! Tracking SDK.\n"
           "LiDAR SDK Version: " RPLIDAR_SDK_VERSION "\n");

    gethostname(hostname, HOST_NAME_MAX);
    host_entry = gethostbyname(hostname);
    address = inet_ntoa(*((struct in_addr*) host_entry->h_addr_list[0]));

    mqtt::async_client client(MQTT_SERVER_ADDRESS, hostname);
    RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if(!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    signal(SIGINT, ctrlc);

    std::thread thScan;
//    std::thread thSub;

    try {
        client.start_consuming();
        auto token = client.connect();
        mqtt::topic scanTopic(client, address, QOS);
        mqtt::topic connectionTopic(client, "$connected", QOS);

        auto response = token->get_connect_response();
        if (!response.is_session_present()) {
            client.subscribe("pingtest", QOS)->wait();
        }

        u_result res;
        if (strcmp(hostname, "pi0") == 0) {
            res = drv->connect("/dev/ttyUSB0", 115200);
        } else {
            res = drv->connect("/dev/ttyUSB0", 256000);
        }
        if(IS_OK(res)) {
            rplidar_response_device_info_t deviceInfo = getDeviceInfo(drv);

            std::stringstream stream;
            char model[8];
            sprintf(model, "%d", deviceInfo.model);
            stream << "+connect:" << hostname << ":" << address << ":" << model;
            std::string s = stream.str();

//            scan(drv, scanTopic);
//            thSub = std::thread(subscription, client);
            connectionTopic.publish(s);

            thScan = std::thread(scan, drv, scanTopic, connectionTopic);
            subscription(client);

        } else {
            fprintf(stderr, "Failed to connect to LIDAR %08x\r\n", res);
        }
    }
    catch (const mqtt::exception& exc) {
        std::cerr << exc << std::endl;
        return 1;
    }

    thScan.join();
//    thSub.join();

    on_finished:
    RPlidarDriver::DisposeDriver(drv);

    return 0;
}
