cmake_minimum_required(VERSION 3.13)
project(rplidar_cpp)

set(CMAKE_CXX_STANDARD 14)

find_package (Threads REQUIRED)

if (WIN32)
    set(ARCH_LIBS vendor/sdk/src/arch/win32/arch_win32.h vendor/sdk/src/arch/win32/net_serial.cpp vendor/sdk/src/arch/win32/net_socket.cpp vendor/sdk/src/arch/win32/net_serial.h vendor/sdk/src/arch/win32/timer.cpp vendor/sdk/src/arch/win32/timer.h vendor/sdk/src/arch/win32/winthread.hpp)
else()
    set(ARCH_LIBS vendor/sdk/src/arch/linux/thread.hpp vendor/sdk/src/arch/linux/arch_linux.h vendor/sdk/src/arch/linux/net_serial.cpp vendor/sdk/src/arch/linux/net_serial.h vendor/sdk/src/arch/linux/net_socket.cpp vendor/sdk/src/arch/linux/timer.cpp vendor/sdk/src/arch/linux/timer.h)
endif(WIN32)

add_executable("${PROJECT_NAME}" main.cpp
        vendor/sdk/include/rplidar.h vendor/sdk/include/rplidar_cmd.h vendor/sdk/include/rptypes.h vendor/sdk/include/rplidar_protocol.h vendor/sdk/include/rplidar_driver.h
        vendor/sdk/src/rplidar_driver.cpp vendor/sdk/src/rplidar_driver_impl.h vendor/sdk/src/rplidar_driver_serial.h vendor/sdk/src/rplidar_driver_TCP.h vendor/sdk/src/sdkcommon.h
        vendor/sdk/src/hal/abs_rxtx.h vendor/sdk/src/hal/assert.h vendor/sdk/src/hal/byteops.h vendor/sdk/src/hal/event.h vendor/sdk/src/hal/locker.h vendor/sdk/src/hal/socket.h vendor/sdk/src/hal/thread.cpp vendor/sdk/src/hal/thread.h vendor/sdk/src/hal/types.h vendor/sdk/src/hal/util.h

        ${ARCH_LIBS}

        # vendor/sdk/src/arch/win32/arch_win32.h vendor/sdk/src/arch/win32/net_serial.cpp vendor/sdk/src/arch/win32/net_socket.cpp vendor/sdk/src/arch/win32/net_serial.h vendor/sdk/src/arch/win32/timer.cpp vendor/sdk/src/arch/win32/timer.h vendor/sdk/src/arch/win32/winthread.hpp

        #vendor/sdk/src/arch/linux/thread.hpp vendor/sdk/src/arch/linux/arch_linux.h vendor/sdk/src/arch/linux/net_serial.cpp vendor/sdk/src/arch/linux/net_serial.h vendor/sdk/src/arch/linux/net_socket.cpp vendor/sdk/src/arch/linux/timer.cpp vendor/sdk/src/arch/linux/timer.h
        )

target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
