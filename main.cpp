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

#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define MINIMUM_POSITION_LIMIT 0    // Refer to the Minimum Position Limit of product eManual
#define MAXIMUM_POSITION_LIMIT 4095 // Refer to the Maximum Position Limit of product eManual
#define BAUDRATE 57600

// DYNAMIXEL Protocol Version (1.0 / 2.0)
// https://emanual.robotis.com/docs/en/dxl/protocol2/
#define PROTOCOL_VERSION 2.0

// Use the actual port assigned to the U2D2.
// ex Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
#define DEVICENAME "/dev/ttyUSB0"

#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0
#define DXL_MOVING_STATUS_THRESHOLD 20 // DYNAMIXEL moving status threshold
#define ESC_ASCII_VALUE 0x1b

#define Y_REST 100
#define Z_REST -85
#define DEGREE_MIN 0
#define DEGREE_MAX 90
#define PI 3.14159
#define R1 45
#define R2 75
#define R3 190

#define NUM_DXL 18

const int DXL_IDS[NUM_DXL] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18};

struct Leg
{
    int motor_ids[3];
    double home_positions[3];
    double move_positions[3];
};

int getch()
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int kbhit(void)
{
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
}

void IK(double x, double y, double z, double thetaList[])
{
    double theta1, theta2, theta3, d, r1, r2, phi1, phi2, phi3;

    y += Y_REST;
    z += Z_REST;

    // printf("IK Input: x=%f, y=%f, z=%f\n", x, y, z); // Debug print

    theta1 = atan2(x, y) * (180 / PI);

    r1 = (sqrt(x * x + y * y)) - R1;

    r2 = z;

    d = sqrt(r1 * r1 + r2 * r2);

    phi1 = (atan2(z, r1)) * (180 / PI);

    // printf("phi1: %lf\n", phi1);

    phi2 = (acos((R2 * R2 + d * d - R3 * R3) / (2 * R2 * d))) * (180 / PI);

    // printf("phi2: %lf\n", phi2);

    theta2 = phi1 + phi2;

    phi3 = (acos((R2 * R2 + R3 * R3 - d * d) / (2 * R2 * R3))) * (180 / PI);

    // printf("phi3: %lf\n", phi3);

    theta3 = phi3 - 90;

    printf("IK Angles in function (degrees): theta1=%f, theta2=%f, theta3=%f\n", theta1, theta2, theta3); // Debug print

    thetaList[0] = theta1;
    thetaList[1] = theta2;
    thetaList[2] = theta3;
}

double bezierPoint(float p0, float p1, float p2, float t)
{
    return pow(1 - t, 2) * p0 + 2 * (1 - t) * t * p1 + pow(t, 2) * p2;
}

int main()
{

    // Setup, communication
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    int dxl_comm_result = COMM_TX_FAIL;

    uint8_t dxl_error = 0;

    int32_t dxl_present_position = 0;

    // Move
    double move_amount[3] = {0};
    double thetaList[3] = {0};

    int goal;

    int home_positions[NUM_DXL];

    double p0x = 0, p0z = 0;
    double p1x = -75, p1z = 75;
    double p2x = -150, p2z = 0;

    const int NUM_LEGS = 6;
    Leg legs[NUM_LEGS] =
        {
            {{1, 2, 3}, {0, 0, 0}, {0, 0, 0}},    // Leg 1
            {{4, 5, 6}, {0, 0, 0}, {0, 0, 0}},    // Leg 2
            {{7, 8, 9}, {0, 0, 0}, {0, 0, 0}},    // Leg 3
            {{10, 11, 12}, {0, 0, 0}, {0, 0, 0}}, // Leg 4
            {{13, 14, 15}, {0, 0, 0}, {0, 0, 0}}, // Leg 5
            {{16, 17, 18}, {0, 0, 0}, {0, 0, 0}}  // Leg 6
        };

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
    for (int i = 0; i < NUM_LEGS; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, legs[i].motor_ids[j], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        }
    }

    while(true)
    {
        printf("Press any key to continue. (Press [ESC] to exit)\n");
        if (getch() == ESC_ASCII_VALUE)
              break;
        // Record Current Positions
        for (int i = 0; i < NUM_LEGS; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, legs[i].motor_ids[j], ADDR_PRESENT_POSITION, (uint32_t *)&dxl_present_position, &dxl_error);
                legs[i].home_positions[j] = dxl_present_position;
                printf("Current Position %d: %d\n", legs[i].motor_ids[j], int(legs[i].home_positions[j]));
            }
        }

        IK(0, 0, 0, thetaList);

        for (int i =0; i < NUM_LEGS; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                legs[i].move_positions[j] = ((thetaList[j] / 360) * 4095) + legs[i].home_positions[j];
                printf("Goal %d: %d\n", legs[i].motor_ids[j], (int)legs[i].move_positions[j]);
                //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, legs[i].motor_ids[j], ADDR_GOAL_POSITION, (int)legs[i].move_positions[j], &dxl_error);
            }
        }

    }
    

    // for (int i = 0; i < NUM_DXL; i++)
    // {
        //goal = (int)move_amount[i];
        
        // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_IDS[i], ADDR_GOAL_POSITION, goal, &dxl_error);

        // if (dxl_comm_result != COMM_SUCCESS)
        // {
        //     printf("Failed to write goal position for Dynamixel ID %d: %s\n", DXL_IDS[i], packetHandler->getTxRxResult(dxl_comm_result));
        // }
        // else if (dxl_error != 0)
        // {
        //     printf("Dynamixel ID %d error: %s\n", DXL_IDS[i], packetHandler->getRxPacketError(dxl_error));
        // }
        // else
        // {
        //     printf("Succeeded writing goal position for Dynamixel ID %d.\n", DXL_IDS[i]);
        // }
   // }

    // Loop
    //   while (true)
    //   {
    //       printf("Press any key to continue. (Press [ESC] to exit)\n");
    //       if (getch() == ESC_ASCII_VALUE)
    //           break;

    //       //Move along bezier curve
    //       for (double dt = 0; dt <= 1; dt += 0.1)
    //       {
    //         double x = bezierPoint(p0x, p1x, p2x, dt);
    //         double z = bezierPoint(p0z, p1z, p2z, dt);
    //         // printf("x = %lf\n", x);
    //         // printf("z = %lf\n", z);
    //         IK(x, 0, z, thetaList);

    //         // Convert angles to move amounts
    //         move_amount[0] = ((thetaList[0] / 360) * 4095) + home_positions[0];
    //         move_amount[1] = ((thetaList[1] / 360) * 4095) + home_positions[1];
    //         move_amount[2] = ((thetaList[2] / 360) * 4095) + home_positions[2];

    //         for (int i = 0; i < NUM_DXL; i++)
    //         {
    //             //dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_IDS[i], ADDR_PRESENT_POSITION, (uint32_t *)&dxl_present_position, &dxl_error);

    //             //printf("Current position for Joint %d: %d\n", DXL_IDS[i], dxl_present_position);

    //             //printf("New Position for Joint %d: %d\n", DXL_IDS[i], (int)move_amount[i]);

    //             goal = (int)move_amount[i];

    //             dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_IDS[i], ADDR_GOAL_POSITION, goal, &dxl_error);

    //         //     if (dxl_comm_result != COMM_SUCCESS)
    //         //     {
    //         //         printf("Failed to write goal position for Dynamixel ID %d: %s\n", DXL_IDS[i], packetHandler->getTxRxResult(dxl_comm_result));
    //         //     }
    //         //     else if (dxl_error != 0)
    //         //     {
    //         //         printf("Dynamixel ID %d error: %s\n", DXL_IDS[i], packetHandler->getRxPacketError(dxl_error));
    //         //     }
    //         //     else
    //         //     {
    //         //         printf("Succeeded writing goal position for Dynamixel ID %d.\n", DXL_IDS[i]);
    //         //     }
    //          }
    //       }

    //       //Move Back to home
    //       for (int new_x = p2x; new_x <= 0; new_x+=10)
    //       {
    //         IK(new_x, 0, 0, thetaList);
    //         move_amount[0] = ((thetaList[0] / 360) * 4095) + home_positions[0];
    //         move_amount[1] = ((thetaList[1] / 360) * 4095) + home_positions[1];
    //         move_amount[2] = ((thetaList[2] / 360) * 4095) + home_positions[2];

    //         for (int i = 0; i < NUM_DXL; i++)
    //         {
    //           //printf("New Position for Joint %d: %d\n", DXL_IDS[i], (int)move_amount[i]);
    //           goal = (int)move_amount[i];
    //           dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_IDS[i], ADDR_GOAL_POSITION, goal, &dxl_error);
    //         }
    //       }

    //   }

    // Disable DYNAMIXEL Torque
    for (int i = 0; i < NUM_DXL; i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_IDS[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
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