/*
 * Copyright 2019-2024 u-blox
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/** @file
 * @brief The application entry point for the NRF52 platform.  Starts
 * the platform and calls Unity to run the selected examples/tests.
 */

// AT — Ова треба да врати OK, што покажува дека комуникацијата е успешна.
// AT+CSQ — За проверка на сигналната јачина.
// AT+CREG? — За проверка на статусот на регистрирање на мрежата.

// u-center (for GNSS monitoring and testing)
// USB Drivers (for your Evaluation Kit or USB-to-Serial Adapter)
// FTDI Drivers (if using FTDI USB-to-Serial adapter): FTDI Drivers

#ifdef U_CFG_OVERRIDE
#include "u_cfg_override.h" // For a customer's configuration override
#endif

#include "stddef.h" // NULL, size_t etc.
#include "stdint.h" // int32_t etc.
#include "stdbool.h"
#include "string.h"
#include "stdlib.h" // getenv

#include "u_cfg_sw.h"
#include "u_cfg_os_platform_specific.h"
#include "u_cfg_app_platform_specific.h"
#include "u_cfg_test_platform_specific.h"

#include "u_error_common.h"

#include "u_port.h"
#include "u_port_os.h"
#include "u_port_debug.h"
#include "u_port_uart.h"
#include "u_gnss_type.h"
#include "u_device_handle.h"
#include "u_gnss.h"
#include "u_gnss_pwr.h"
#include "u_cell_loc.h"
#include "u_location.h"
#include "u_gnss_pos.h"
#include "u_at_client.h"
#include "u_device.h"
#include "u_cell.h"
#include "u_cell_pwr.h"
#include "u_cell_net.h"
#include "u_cell.h"
#include "u_device_handle.h"
#include "u_device.h"
// #include "u_gnss_cfg_val_key.h"
// #include"u_gnss_cfg.h"
// #include"u_gnss_dec_ubx_nav_hpposllh.h"
// #include"u_gnss_dec_ubx_nav_pvt.h"
// #include"u_gnss_dec.h"
// #include"u_gnss_geofence.h"
// #include"u_gnss_info.h"
// #include"u_gnss_mga.h"
// #include"u_gnss_module_type.h"
// #include"u_gnss_msg.h"
#include "u_gnss_pos.h"
#include "u_gnss_pwr.h"
#include "u_gnss_type.h"
#include "u_gnss_util.h"
#include "u_gnss_private.h"

#include "u_debug_utils.h"
#include "u_network_config_ble.h"
#include "u_network_config_cell.h"
#include "u_network_config_gnss.h"
#include "u_network_config_wifi.h"
#include "u_network_type.h"
#include "u_network.h"
#include "u_sock.h"

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

/* ----------------------------------------------------------------
 * VARIABLES
 * -------------------------------------------------------------- */

// This is intentionally a bit hidden and comes from u_port_debug.c
extern int32_t gStdoutCounter;

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS
 * -------------------------------------------------------------- */

// // Send an AT command to the SARA module via UART
// static void sendAtCommand(int32_t uartHandle, const char *command)
// {
//     uPortLog("Sending AT command: %s\n", command);
//     uPortUartWrite(uartHandle, command, strlen(command));

//     // Small delay for the module to process the command
//     uPortTaskBlock(1000); // 1 second delay

//     char response[256]; // Buffer to store the response
//     int32_t bytesRead = uPortUartRead(uartHandle, response, sizeof(response) - 1);

//     if (bytesRead > 0) {
//         response[bytesRead] = '\0'; // Null-terminate the response
//         uPortLog("Received response: %s\n", response);
//     } else {
//         uPortLog("No response from the module.\n");
//     }
// }

static const uDeviceCfg_t gDeviceCfg = {
    .deviceType = U_DEVICE_TYPE_CELL,
    .deviceCfg =
        {
            .cfgCell = {.moduleType =
                            U_CELL_MODULE_TYPE_SARA_R422, // Define this in your build system
                        .pSimPinCode = NULL,              // Set SIM PIN if needed
                        .pinEnablePower = U_CFG_APP_PIN_CELL_ENABLE_POWER,
                        .pinPwrOn = U_CFG_APP_PIN_CELL_PWR_ON,
                        .pinVInt = U_CFG_APP_PIN_CELL_VINT,
                        .pinDtrPowerSaving = U_CFG_APP_PIN_CELL_DTR},
        },
    .transportType = U_DEVICE_TRANSPORT_TYPE_UART,
    .transportCfg =
        {
            .cfgUart =
                {
                    .uart = 7, // This should match your UART, e.g., 7 for COM7
                    .baudRate = U_CELL_UART_BAUD_RATE, // Define your baud rate, e.g., 115200
                    .pinTxd = U_CFG_APP_PIN_CELL_TXD,
                    .pinRxd = U_CFG_APP_PIN_CELL_RXD,
                    .pinCts = U_CFG_APP_PIN_CELL_CTS,
                    .pinRts = U_CFG_APP_PIN_CELL_RTS,
                    .pPrefix = NULL // For Linux setups; can be NULL for embedded systems
                },
        },
};

static const uNetworkCfgCell_t gNetworkCfg = {
    .type = U_NETWORK_TYPE_CELL, .pApn = "data.lycamobile.mk", .timeoutSeconds = 240};

#define MY_SERVER_NAME "tcpbin.com"
#define MY_SERVER_PORT 4242

void logAtCommandResponse(int32_t uartHandle)
{
    char buffer[256]; // Buffer to store the response
    int32_t bytesRead = uPortUartRead(uartHandle, buffer, sizeof(buffer) - 1);

    uPortLog("Reading AT command response...\n");
    if (bytesRead > 0) {
        buffer[bytesRead] = '\0';
        uPortLog("Received response: %s\n", buffer);
    } else {
        uPortLog("No response from the module.\n");
    }
}

static void appTask(void *pParam)
{
    (void)pParam;
    uDeviceHandle_t gnssHandle = NULL;
    uDeviceHandle_t cellHandle = NULL;
    // int32_t latitudeX1e7;
    // int32_t longitudeX1e7;
    uDeviceHandle_t devHandle;
    int32_t sock;
    uSockAddress_t remoteAddress;
    int32_t x;
    // uGnssTransportHandle_t transportHandle;
    const char *pMessage = "Hello server\r\n";

    // #define U_CFG_APP_CELL_BAUD_RATE 115200
#define U_CFG_APP_CELL_UART_BUFFER_SIZE 1024

#if U_CFG_TEST_ENABLE_INACTIVITY_DETECTOR
    uDebugUtilsInitInactivityDetector(&gStdoutCounter);
#endif

#ifdef U_CFG_MUTEX_DEBUG
    uMutexDebugInit();
    uMutexDebugWatchdog(uMutexDebugPrint, NULL, U_MUTEX_DEBUG_WATCHDOG_TIMEOUT_SECONDS);
#endif

    if (uPortInit() == 0) {
        uPortLog("Port initialized successfully.\n");
    } else {
        uPortLog("Failed to initialize port.\n");
        return;
    }
    // Initialize GNSS
    if (uGnssInit() != 0) {
        uPortLog("Failed to initialize GNSS.\n");
        return;
    } else {
        uPortLog("GNSS is initialize successfully \n");
    }
    if (uDeviceInit() != 0) {
        uPortLog("Failed to initialize ubxlib device.\n");
        return;
    } else {
        uPortLog("Device is initialize successfully \n");
    }

    if (uCellInit() != 0) {
        uPortLog("Failed to initialize ubxlib cell.\n");
        return;
    } else {
        uPortLog("Cell is initialize successfully \n");
    }

    uPortLog("\n\nU_APP: application task started.\n");

    UNITY_BEGIN();

    x = uDeviceOpen(&gDeviceCfg, &devHandle);
    uPortLog("## Opened device with return code %d.\n", x);

    if (x == 0) {
        uPortLog("Device opened successfully.\n");
    } else {
        uPortLog("Failed to open device.\n");
    }

    if (uNetworkInterfaceUp(devHandle, U_NETWORK_TYPE_CELL, &gNetworkCfg) == 0) {
        uPortLog("Network interface is up.\n");
    } else {
        uPortLog("Failed to bring up the network interface.\n");
    }

    if (uSockGetHostByName(devHandle, MY_SERVER_NAME, &(remoteAddress.ipAddress)) == 0) {
        uPortLog("Resolved hostname: %s\n", MY_SERVER_NAME);
        remoteAddress.port = MY_SERVER_PORT;

        // Create a TCP socket
        sock = uSockCreate(devHandle, U_SOCK_TYPE_STREAM, U_SOCK_PROTOCOL_TCP);
        if (sock >= 0) {
            uPortLog("Socket created successfully.\n");

            // Connect the socket to the remote server
            if (uSockConnect(sock, &remoteAddress) == 0) {
                uPortLog("Connected to server %s on port %d.\n", MY_SERVER_NAME, MY_SERVER_PORT);

                // Send data to the server
                size_t messageSize;
                size_t sentSize = 0;
                messageSize = strlen(pMessage);
                while (sentSize < messageSize) {
                    int bytesSent = uSockWrite(sock, (void *)(pMessage + sentSize), messageSize - sentSize);
                    if (bytesSent > 0) {
                        sentSize += bytesSent;
                    } else if (bytesSent < 0) {
                        uPortLog("Error sending message: %d\n", x);
                        break; // Exit the loop on error
                    }
                }
                uPortLog("Message sent: %s\n", pMessage);
                uPortTaskBlock(2000);
                char response[1024];
                int bytesRead = uSockRead(sock, response, sizeof(response) - 1);
                if (bytesRead > 0) {
                    response[bytesRead] = '\0'; 
                    uPortLog("Received response: %s\n", response);
                } else {
                    uPortLog("No response or error reading from server.\n");
                }
                // Close the socket
                uSockClose(sock);
                uSockCleanUp(sock);
            } else {
                uPortLog("Failed to connect the socket.\n");
            }
            // Bring the network interface down after use
            uNetworkInterfaceDown(devHandle, U_NETWORK_TYPE_CELL);
        } else {
            uPortLog("Failed to create the socket.\n");
        }

        // Close the device and clean up resources
        if (uDeviceClose(devHandle, true) != 0) {
            // Device did not respond to power off; force close
            uDeviceClose(devHandle, false);
        }

       

        //     uPortLog("\n\nU_APP: application task ended.\n");
        //     uPortDeinit();
        uGnssPwrOff(devHandle);
        uNetworkInterfaceDown(devHandle, U_NETWORK_TYPE_CELL);

    } else {
        uPortLog("Failed to bring up the socket up.\n");
    }

    if (uDeviceClose(devHandle, true) != 0) {
        // Device did not respond to power off; force close
        uDeviceClose(devHandle, false);
    }
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * -------------------------------------------------------------- */

// Unity setUp() function.
void setUp(void)
{
    // Nothing to do
}

// Unity tearDown() function.
void tearDown(void)
{
    // Nothing to do
}

void testFail(void)
{
    // Nothing to do
}

// Entry point
int main(void)
{
    // Start the platform to run the tests
    return uPortPlatformStart(appTask, NULL, U_CFG_OS_APP_TASK_STACK_SIZE_BYTES,
                              U_CFG_OS_APP_TASK_PRIORITY);
}
