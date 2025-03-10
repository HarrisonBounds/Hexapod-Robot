

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

#include "dynamixel_sdk.h"

#include <unistd.h>

#define X_SERIES

// Control table address

#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define BAUDRATE 57600

#define PROTOCOL_VERSION 2.0

#define DEVICENAME "/dev/ttyUSB0"

#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0
#define DXL_MOVING_STATUS_THRESHOLD 20
#define ESC_ASCII_VALUE 0x1b

#define Y_REST 50
#define Z_REST -110
#define DEGREE_MAX 360.0
#define POSITION_MAX 4095.0
#define PI 3.14159
#define R1 50
#define R2 100
#define R3 190

#define NUM_DXL 18
#define NUM_LEGS 6

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

    theta1 = atan2(x, y) * (180 / PI);

    r1 = (sqrt(x * x + y * y)) - R1;

    r2 = z;

    d = sqrt(r1 * r1 + r2 * r2);

    phi1 = (atan2(z, r1)) * (180 / PI);

    phi2 = (acos((R2 * R2 + d * d - R3 * R3) / (2 * R2 * d))) * (180 / PI);

    theta2 = phi1 + phi2;

    phi3 = (acos((R2 * R2 + R3 * R3 - d * d) / (2 * R2 * R3))) * (180 / PI);

    theta3 = phi3 - 90;

    // printf("IK Angles in function (degrees): theta1=%f, theta2=%f, theta3=%f\n", theta1, theta2, theta3); // Debug print

    thetaList[0] = theta1;
    thetaList[1] = theta2;
    thetaList[2] = theta3;
}

double bezierPoint(float p0, float p1, float p2, float t)
{
    return pow(1 - t, 2) * p0 + 2 * (1 - t) * t * p1 + pow(t, 2) * p2;
}

void move(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler, dynamixel::GroupSyncWrite &groupSyncWrite, Leg *legs, int leg_indices[], int array_len)
{
    // int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;
    int32_t dxl_present_position = 0;

    uint8_t param_goal_position[NUM_DXL];
    bool dxl_addparam_result;

    groupSyncWrite.clearParam();

    // Add goal positions for all motors in the specified legs
    for (int i = 0; i < array_len; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            int goal_position = (int)legs[leg_indices[i]].move_positions[j];

            // Pack the 32-bit goal position into 4 bytes
            param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goal_position));
            param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goal_position));
            param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goal_position));
            param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goal_position));

            // Add the motor to the sync group
            dxl_addparam_result = groupSyncWrite.addParam(legs[leg_indices[i]].motor_ids[j], param_goal_position);
        }
    }

    int dxl_comm_result = groupSyncWrite.txPacket();
    groupSyncWrite.clearParam();
}

void calculatePosition(Leg *legs, double thetaList[], int leg_indices[], int array_len)
{
    for (int i = 0; i < array_len; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            legs[leg_indices[i]].move_positions[j] = legs[leg_indices[i]].home_positions[j] - ((thetaList[j] / DEGREE_MAX) * POSITION_MAX);
        }
    }
}

void tripodGait(int &tripod_x, double thetaList[], Leg *legs, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler, dynamixel::GroupSyncWrite &groupSyncWrite, int tripod_indices1[], int tripod_indices_length1, int tripod_indices1a[], int tripod_indices_length1a, int tripod_indices2[], int tripod_indices_length2, int tripod_indices2a[], int tripod_indices_length2a, int home_indices[], int home_indices_length, int start)
{

    int tripod1_x = 0;
    int tripod1_z = 0;
    int tripod1a_x = 0;
    int tripod1a_z = 0;
    int tripod2_x = 0;
    int tripod2_z = 0;
    int tripod2a_x = 0;
    int tripod2a_z = 0;

    for (double t = 0; t <= 1; t += 0.01)
    {
        tripod1_x = bezierPoint(-start, tripod_x / 2, tripod_x, t);
        tripod1_z = bezierPoint(0, 40, 0, t);
        IK(tripod1_x, 0, tripod1_z, thetaList);
        calculatePosition(legs, thetaList, tripod_indices1, tripod_indices_length1);

        tripod1a_x = bezierPoint(start, tripod_x / 2, -tripod_x, t);
        tripod1a_z = bezierPoint(0, 40, 0, t);
        IK(tripod1a_x, 0, tripod1a_z, thetaList);
        calculatePosition(legs, thetaList, tripod_indices1a, tripod_indices_length1a);

        tripod2_x = bezierPoint(-start, tripod_x / 2, tripod_x, t);
        IK(tripod2_x, 0, 0, thetaList);
        calculatePosition(legs, thetaList, tripod_indices2, tripod_indices_length2);

        tripod2a_x = bezierPoint(start, tripod_x / 2, -tripod_x, t);
        IK(tripod2a_x, 0, 0, thetaList);
        calculatePosition(legs, thetaList, tripod_indices2a, tripod_indices_length2a);

        move(packetHandler, portHandler, groupSyncWrite, legs, home_indices, home_indices_length);

        usleep(10000);
    }

    for (double t = 0; t <= 1; t += 0.01)
    {
        tripod1_x = bezierPoint(tripod_x, tripod_x / 2, -tripod_x, t);
        IK(tripod1_x, 0, 0, thetaList);
        calculatePosition(legs, thetaList, tripod_indices1, tripod_indices_length1);

        tripod1a_x = bezierPoint(-tripod_x, tripod_x / 2, tripod_x, t);
        IK(tripod1a_x, 0, 0, thetaList);
        calculatePosition(legs, thetaList, tripod_indices1a, tripod_indices_length1a);

        tripod2_x = bezierPoint(tripod_x, tripod_x / 2, -tripod_x, t);
        tripod2_z = bezierPoint(0, 40, 0, t);
        IK(tripod2_x, 0, tripod2_z, thetaList);
        calculatePosition(legs, thetaList, tripod_indices2, tripod_indices_length2);

        tripod2a_x = bezierPoint(-tripod_x, tripod_x / 2, tripod_x, t);
        tripod2a_z = bezierPoint(0, 40, 0, t);
        IK(tripod2a_x, 0, tripod2a_z, thetaList);
        calculatePosition(legs, thetaList, tripod_indices2a, tripod_indices_length2a);

        move(packetHandler, portHandler, groupSyncWrite, legs, home_indices, home_indices_length);

        usleep(10000);
    }

    start = tripod_x;
}

void turning(int &start, double thetaList[], Leg *legs, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler, dynamixel::GroupSyncWrite &groupSyncWrite, int tripod_indices1[], int tripod_indices_length1, int tripod_indices2[], int tripod_indices_length2, int home_indices[], int home_indices_length, int tripod_x)
{
    double turn1_x;
    double turn1_z;
    double turn2_x;
    double turn2_z;

    for (double t = 0; t <= 1; t += 0.01)
    {
        turn1_x = bezierPoint(start, tripod_x / 2, -tripod_x, t);
        turn1_z = bezierPoint(0, 30, 0, t);
        IK(turn1_x, 0, turn1_z, thetaList);
        calculatePosition(legs, thetaList, tripod_indices1, tripod_indices_length1);

        turn2_x = bezierPoint(-start, tripod_x / 2, tripod_x, t);
        IK(turn2_x, 0, 0, thetaList);
        calculatePosition(legs, thetaList, tripod_indices2, tripod_indices_length2);

        move(packetHandler, portHandler, groupSyncWrite, legs, home_indices, home_indices_length);

        usleep(10000);
    }

    
    for (double t = 0; t <= 1; t += 0.01)
    {
        turn1_x = bezierPoint(-tripod_x, tripod_x / 2, tripod_x, t);
        IK(turn1_x, 0, 0, thetaList);
        calculatePosition(legs, thetaList, tripod_indices1, tripod_indices_length1);

        turn2_x = bezierPoint(tripod_x, tripod_x / 2, -tripod_x, t);
        turn2_z = bezierPoint(0, 30, 0, t);
        IK(turn2_x, 0, turn2_z, thetaList);
        calculatePosition(legs, thetaList, tripod_indices2, tripod_indices_length2);

        move(packetHandler, portHandler, groupSyncWrite, legs, home_indices, home_indices_length);

        usleep(10000);
    }

    start = tripod_x;
}

void wave(double thetaList[], Leg *legs, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler, dynamixel::GroupSyncWrite &groupSyncWrite, int wave_leg[], int wave_leg_length)
{
    // Use Bezier curves for smoother waving motion
    for (double t = 0; t <= 1; t += 0.02)
    {
        double wave_x = bezierPoint(-200, -100, 0, t);
        double wave_y = bezierPoint(150, 170, 150, t);
        double wave_z = bezierPoint(150, 170, 150, t);
        
        IK(wave_x, wave_y, wave_z, thetaList);
        calculatePosition(legs, thetaList, wave_leg, wave_leg_length);
        move(packetHandler, portHandler, groupSyncWrite, legs, wave_leg, wave_leg_length);
        
        usleep(10000); // Shorter sleep for smoother animation
    }

    // Wave back to starting position
    for (double t = 0; t <= 1; t += 0.02)
    {
        double wave_x = bezierPoint(0, -80, -160, t);
        double wave_y = bezierPoint(150, 170, 150, t);
        double wave_z = bezierPoint(150, 170, 150, t);
        
        IK(wave_x, wave_y, wave_z, thetaList);
        calculatePosition(legs, thetaList, wave_leg, wave_leg_length);
        move(packetHandler, portHandler, groupSyncWrite, legs, wave_leg, wave_leg_length);
        
        usleep(10000);
    }

    for (double t = 0; t <= 1; t += 0.02)
    {
        double wave_x = bezierPoint(-160, -60, -40, t);
        double wave_y = bezierPoint(150, 170, 150, t);
        double wave_z = bezierPoint(150, 170, 150, t);
        IK(wave_x, wave_y, wave_z, thetaList);
        calculatePosition(legs, thetaList, wave_leg, wave_leg_length);
        move(packetHandler, portHandler, groupSyncWrite, legs, wave_leg, wave_leg_length);
        usleep(10000);
    }

    for (double t = 0; t <= 1; t += 0.02)
    {
        double wave_x = bezierPoint(-40, -30, -100, t);
        double wave_y = bezierPoint(150, 170, 150, t);
        double wave_z = bezierPoint(150, 170, 150, t);
        IK(wave_x, wave_y, wave_z, thetaList);
        calculatePosition(legs, thetaList, wave_leg, wave_leg_length);
        move(packetHandler, portHandler, groupSyncWrite, legs, wave_leg, wave_leg_length);
        usleep(10000);
    }
}

int main()
{
    dynamixel::PortHandler::getPortHandler(DEVICENAME);

    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 4);

    int dxl_comm_result = COMM_TX_FAIL;

    uint8_t dxl_error = 0;

    int32_t dxl_present_position = 0;

    // Move
    double thetaList[3] = {0};

    int goal;

    // All legs
    int home_indices[6] = {0, 1, 2, 3, 4, 5};
    int home_indices_length = sizeof(home_indices) / sizeof(home_indices[0]);

    // Calculate leg movement for tripod gait
    int tripod_indices1[3] = {0, 2, 4};
    int tripod_indices2[3] = {1, 3, 5};
    int tripod_indices1a[1] = {4};
    int tripod_indices2a[1] = {1};
    int tripod_indices_length1 = sizeof(tripod_indices1) / sizeof(tripod_indices1[0]);
    int tripod_indices_length2 = sizeof(tripod_indices2) / sizeof(tripod_indices2[0]);
    int tripod_indices_length1a = sizeof(tripod_indices1a) / sizeof(tripod_indices1a[0]);
    int tripod_indices_length2a = sizeof(tripod_indices2a) / sizeof(tripod_indices2a[0]);

    int wave_leg[1] = {0};
    int wave_leg_length = sizeof(wave_leg) / sizeof(wave_leg[0]);

    double turn1_x;
    double turn1_z;
    double turn2_x;
    double turn2_z;

    int tripod_x = 30; // Step size in x direction

    double timestep = 0.01;

    int start = 0;

    Leg legs[NUM_LEGS] =
        {
            {{1, 2, 3}, {2000, 2080, 2055}, {0, 0, 0}},
            {{4, 5, 6}, {2100, 2067, 2033}, {0, 0, 0}},    // 2216, 2067, 2033
            {{7, 8, 9}, {2100, 2065, 2003}, {0, 0, 0}},    // 2100, 2065, 2003
            {{10, 11, 12}, {2100, 2055, 2069}, {0, 0, 0}}, // 2106, 2055, 2069
            {{13, 14, 15}, {2101, 2046, 2075}, {0, 0, 0}}, // 2101, 2046, 2075
            {{16, 17, 18}, {2000, 2043, 2057}, {0, 0, 0}}  // 1989, 2043, 2057
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

    // Move to the Home Position
    IK(0, 0, 0, thetaList);

    calculatePosition(legs, thetaList, home_indices, home_indices_length);

    move(packetHandler, portHandler, groupSyncWrite, legs, home_indices, home_indices_length);

    while (true)
    {
        printf("Press any key to continue. (Press [ESC] to exit)\n");
        if (getch() == ESC_ASCII_VALUE)
            break;

        wave(thetaList, legs, packetHandler, portHandler, groupSyncWrite, wave_leg, wave_leg_length);


        // IK(-180, 150, 150, thetaList);
        // calculatePosition(legs, thetaList, wave_leg, wave_leg_length);
        // move(packetHandler, portHandler, groupSyncWrite, legs, wave_leg, wave_leg_length);

        // usleep(500000);

        // IK(-30, 150, 150, thetaList);
        // calculatePosition(legs, thetaList, wave_leg, wave_leg_length);
        // move(packetHandler, portHandler, groupSyncWrite, legs, wave_leg, wave_leg_length);

        // usleep(500000);

        // IK(-180, 150, 150, thetaList);
        // calculatePosition(legs, thetaList, wave_leg, wave_leg_length);
        // move(packetHandler, portHandler, groupSyncWrite, legs, wave_leg, wave_leg_length);

        // usleep(500000);

        // IK(-30, 150, 150, thetaList);
        // calculatePosition(legs, thetaList, wave_leg, wave_leg_length);
        // move(packetHandler, portHandler, groupSyncWrite, legs, wave_leg, wave_leg_length);

        // usleep(500000);

    }

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
