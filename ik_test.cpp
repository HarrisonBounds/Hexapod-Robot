/*******************************************************************************
 * Copyright 2017 ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

/*******************************************************************************
************************     Read and Write Example      ***********************
* Required Environment to run this example :
*   - Protocol 2.0 supported DYNAMIXEL(X, P, PRO/PRO(A), MX 2.0 series)
*   - DYNAMIXEL Starter Set (U2D2, U2D2 PHB, 12V SMPS)
* How to use the example :
*   - Use proper DYNAMIXEL Model definition from line #44
*   - Build and Run from proper architecture subdirectory.
*   - For ARM based SBCs such as Raspberry Pi, use linux_sbc subdirectory to build and run.
*   - https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/
* Author: Ryu Woon Jung (Leon)
* Maintainer : Zerom, Will Son
*******************************************************************************/

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <algorithm>

#include "dynamixel_sdk.h" // Uses DYNAMIXEL SDK library

/********* DYNAMIXEL Model definition *********
***** (Use only one definition at a time) *****/
#define X_SERIES // X330, X430, X540, 2X430
// #define PRO_SERIES // H54, H42, M54, M42, L54, L42
// #define PRO_A_SERIES // PRO series with (A) firmware update.
// #define P_SERIES  // PH54, PH42, PM54
// #define XL320  // [WARNING] Operating Voltage : 7.4V
// #define MX_SERIES // MX series with 2.0 firmware update.
// #define Y_SERIES // Y70, Y80

// Control table address
#if defined(X_SERIES) || defined(MX_SERIES)
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define MINIMUM_POSITION_LIMIT 0    // Refer to the Minimum Position Limit of product eManual
#define MAXIMUM_POSITION_LIMIT 4095 // Refer to the Maximum Position Limit of product eManual
#define BAUDRATE 57600
#elif defined(PRO_SERIES)
#define ADDR_TORQUE_ENABLE 562 // Control table address is different in DYNAMIXEL model
#define ADDR_GOAL_POSITION 596
#define ADDR_PRESENT_POSITION 611
#define MINIMUM_POSITION_LIMIT -150000 // Refer to the Minimum Position Limit of product eManual
#define MAXIMUM_POSITION_LIMIT 150000  // Refer to the Maximum Position Limit of product eManual
#define BAUDRATE 57600
#elif defined(P_SERIES) || defined(PRO_A_SERIES)
#define ADDR_TORQUE_ENABLE 512 // Control table address is different in DYNAMIXEL model
#define ADDR_GOAL_POSITION 564
#define ADDR_PRESENT_POSITION 580
#define MINIMUM_POSITION_LIMIT -150000 // Refer to the Minimum Position Limit of product eManual
#define MAXIMUM_POSITION_LIMIT 150000  // Refer to the Maximum Position Limit of product eManual
#define BAUDRATE 57600
#elif defined(XL320)
#define ADDR_TORQUE_ENABLE 24
#define ADDR_GOAL_POSITION 30
#define ADDR_PRESENT_POSITION 37
#define MINIMUM_POSITION_LIMIT 0    // Refer to the CW Angle Limit of product eManual
#define MAXIMUM_POSITION_LIMIT 1023 // Refer to the CCW Angle Limit of product eManual
#define BAUDRATE 1000000            // Default Baudrate of XL-320 is 1Mbps
#elif defined(Y_SERIES)
#define ADDR_TORQUE_ENABLE 512 // Control table address is different in DYNAMIXEL model
#define ADDR_GOAL_POSITION 532
#define ADDR_PRESENT_POSITION 552
#define MINIMUM_POSITION_LIMIT -262144 // Refer to the Minimum Position Limit of product eManual
#define MAXIMUM_POSITION_LIMIT 262144  // Refer to the Maximum Position Limit of product eManual
#define BAUDRATE 57600
#endif

// DYNAMIXEL Protocol Version (1.0 / 2.0)
// https://emanual.robotis.com/docs/en/dxl/protocol2/
#define PROTOCOL_VERSION 2.0

// Factory default ID of all DYNAMIXEL is 1
#define DXL_ID 1

// Use the actual port assigned to the U2D2.
// ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
#define DEVICENAME "/dev/ttyUSB0"

#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0
#define DXL_MOVING_STATUS_THRESHOLD 20 // DYNAMIXEL moving status threshold
#define ESC_ASCII_VALUE 0x1b

#define Y_REST 185
#define Z_REST -70
#define DEGREE_MIN 0
#define DEGREE_MAX 90
#define PI 3.14159
#define R1 50
#define R2 114
#define R3 190

#define NUM_DXL 3



const double home_x = 0;      // X coordinate of the foot in the home position
const double home_y = Y_REST; // Y coordinate of the foot in the home position
const double home_z = Z_REST; // Z coordinate of the foot in the home position

const int DXL_IDS[NUM_DXL] = {1, 2, 3};

int getch()
{
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
#elif defined(_WIN32) || defined(_WIN64)
    return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
#elif defined(_WIN32) || defined(_WIN64)
    return _kbhit();
#endif
}

void IK(double x, double y, double z, double thetaList[])
{

    double theta1, theta2, theta3, d, r1, r2, phi1, phi2, phi3;

    y += Y_REST;
    z += Z_REST;

    printf("IK Input: x=%f, y=%f, z=%f\n", x, y, z); // Debug print

    theta1 = atan2(x, y) * (180 / PI);

    r1 = (sqrt(x * x + y * y)) - R1;

    r2 = z;

    d = sqrt(r1 * r1 + r2 * r2);

    phi1 = (atan2(z, r1)) * (180 / PI);

    printf("phi1: %lf\n", phi1);

    phi2 = (acos((R2 * R2 + d * d - R3 * R3) / (2 * R2 * d))) * (180 / PI);

    printf("phi2: %lf\n", phi2);

    theta2 = phi1 + phi2;

    phi3 = (acos((R2 * R2 + R3 * R3 - d * d) / (2 * R2 * R3))) * (180 / PI);

    printf("phi3: %lf\n", phi3);

    theta3 = phi3-90;

    printf("IK Angles in function (degrees): theta1=%f, theta2=%f, theta3=%f\n", theta1, theta2, theta3); // Debug print

    thetaList[0] = theta1;
    thetaList[1] = theta2;
    thetaList[2] = theta3;
}

void FK(double theta1, double theta2, double theta3) {
    double x, y, z;
    // Convert angles from degrees to radians
    double theta1_rad = theta1 * (PI / 180.0);
    double theta2_rad = theta2 * (PI / 180.0);
    double theta3_rad = theta3 * (PI / 180.0);

    // Calculate foot position
    x = (R1 + R2 * cos(theta2_rad) + R3 * cos(theta2_rad + theta3_rad)) * sin(theta1_rad);
    y = (R1 + R2 * cos(theta2_rad) + R3 * cos(theta2_rad + theta3_rad)) * cos(theta1_rad);
    z = R2 * sin(theta2_rad) + R3 * sin(theta2_rad + theta3_rad);

    printf("FK Position: x=%f, y=%f, z=%f\n", x, y, z);
}

double bezierPoint(float p0, float p1, float p2, float t)
{
    return pow(1 - t, 2) * p0 + 2 * (1 - t) * t * p1 + pow(t, 2) * p2;
}

int main()
{
    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // int index = 0;
    int dxl_comm_result = COMM_TX_FAIL; // Communication result
    // int dxl_goal_position[2] = {MINIMUM_POSITION_LIMIT, MAXIMUM_POSITION_LIMIT};         // Goal position

    uint8_t dxl_error = 0; // DYNAMIXEL error
#if defined(XL320)
    int16_t dxl_present_position = 0; // XL-320 uses 2 byte Position data
#else
    int32_t dxl_present_position = 0; // Read 4 byte Position data
#endif

    // Open port
    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    // Enable DYNAMIXEL Torque
    for (int i = 0; i < NUM_DXL; i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_IDS[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            printf("Succeeded enabling DYNAMIXEL Torque.\n");
        }
    }

    int home_positions[NUM_DXL];

    for (int i = 0; i < NUM_DXL; i++)
    {
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, i + 1, ADDR_PRESENT_POSITION, (uint32_t *)&dxl_present_position, &dxl_error);
        home_positions[i] = dxl_present_position;
        printf("Joint %d Angle: %d\n", i + 1, (int)((dxl_present_position / 4095.0) * 360.0));    
        printf("Joint %d Position at Position (Initial): %d\n", i + 1, dxl_present_position);
    }

    double move_amount[3] = {0};
    double thetaList[3] = {0};

    double x, y, z;
    double move_angle;
    int goal;

    IK(0, 0, 0, thetaList);


    move_amount[0] = ((thetaList[0] / 360) * 4095) + home_positions[0];
    move_amount[1] = ((thetaList[1] / 360) * 4095) + home_positions[1];
    move_amount[2] = ((thetaList[2] / 360) * 4095) + home_positions[2]; 
    
    for (int i = 0; i < NUM_DXL; i++)
    {
        goal = (int)move_amount[i];
        printf("Home position for Joint %d: %d\n", i + 1, goal);
        //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_IDS[i], ADDR_GOAL_POSITION, goal, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("Failed to write goal position for Dynamixel ID %d: %s\n", DXL_IDS[i], packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("Dynamixel ID %d error: %s\n", DXL_IDS[i], packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            printf("Succeeded writing goal position for Dynamixel ID %d.\n", DXL_IDS[i]);
        }
    }


    while (true)
    {
        printf("Press any key to continue. (Press [ESC] to exit)\n");
        if (getch() == ESC_ASCII_VALUE)
            break;
        else
        {
            scanf("%lf %lf %lf", &x, &y, &z);
        }

        IK(x, y, z, thetaList);

        // Convert angles to move amounts
        move_amount[0] = ((thetaList[0] / 360) * 4095) - home_positions[0];
        move_amount[1] = ((thetaList[1] / 360) * 4095) - home_positions[1];
        move_amount[2] = ((thetaList[2] / 360) * 4095) - home_positions[2];


        for (int i = 0; i < NUM_DXL; i++)
        {
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_IDS[i], ADDR_PRESENT_POSITION, (uint32_t *)&dxl_present_position, &dxl_error);

            printf("Current position for Joint %d: %d\n", DXL_IDS[i], dxl_present_position);

            printf("New Position for Joint %d: %d\n", DXL_IDS[i], (int)move_amount[i]);

            goal = (int)move_amount[i];

            //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_IDS[i], ADDR_GOAL_POSITION, goal, &dxl_error);

            if (dxl_comm_result != COMM_SUCCESS)
            {
                printf("Failed to write goal position for Dynamixel ID %d: %s\n", DXL_IDS[i], packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0)
            {
                printf("Dynamixel ID %d error: %s\n", DXL_IDS[i], packetHandler->getRxPacketError(dxl_error));
            }
            else
            {
                printf("Succeeded writing goal position for Dynamixel ID %d.\n", DXL_IDS[i]);
            }
        }
    }

    // Disable DYNAMIXEL Torque
    for (int i = 0; i < NUM_DXL; i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }
        else
        {
            printf("Succeeded disabling DYNAMIXEL Torque.\n");
        }
    }

    // Close port
    portHandler->closePort();
    return 0;
}