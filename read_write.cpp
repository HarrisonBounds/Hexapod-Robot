
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0

#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk.h"  // Uses DYNAMIXEL SDK library

#define X_SERIES 

#define ADDR_TORQUE_ENABLE          64
#define ADDR_GOAL_POSITION          116
#define ADDR_PRESENT_POSITION       132
#define MINIMUM_POSITION_LIMIT      0  // Refer to the Minimum Position Limit of product eManual
#define MAXIMUM_POSITION_LIMIT      2000  // Refer to the Maximum Position Limit of product eManual
#define BAUDRATE                    57600


// DYNAMIXEL Protocol Version (1.0 / 2.0)
// https://emanual.robotis.com/docs/en/dxl/protocol2/
#define PROTOCOL_VERSION  2.0

// Factory default ID of all DYNAMIXEL is 1
#define NUM_DXL 3
const u_int8_t DXL_IDS[NUM_DXL] = {1, 2, 3};

// Use the actual port assigned to the U2D2.
// ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
#define DEVICENAME  "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0
#define DXL_MOVING_STATUS_THRESHOLD     20  // DYNAMIXEL moving status threshold
#define ESC_ASCII_VALUE                 0x1b

int getch() {
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

int kbhit(void) {
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

  if (ch != EOF) {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

int main() {
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int index = 0;
  int dxl_comm_results[NUM_DXL] = {COMM_TX_FAIL, COMM_TX_FAIL, COMM_TX_FAIL};             // Communication result
  int dxl_goal_position[2] = {MINIMUM_POSITION_LIMIT, MAXIMUM_POSITION_LIMIT};         // Goal position

  uint8_t dxl_error = 0;                          // DYNAMIXEL error
  int32_t dxl_present_position = 0;  // Read 4 byte Position data


  // Open port
  if (portHandler->openPort()) {
    printf("Succeeded to open the port!\n");
  }
  else {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE)) {
    printf("Succeeded to change the baudrate!\n");
  }
  else {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Enable DYNAMIXEL Torque
  for(int i =0; i < NUM_DXL; i++)
  {
    printf("For Servo %d", i);
    dxl_comm_results[i] = packetHandler->write1ByteTxRx(portHandler, DXL_IDS[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_results[i] != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_results[i]));
    }
    else if (dxl_error != 0) {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else {
      printf("Succeeded enabling DYNAMIXEL Torque.\n");
    }
  }
  

  //Start Process
  while(1) {
    printf("Press any key to continue. (Press [ESC] to exit)\n");
    if (getch() == ESC_ASCII_VALUE)
      break;

    // Write goal position
    for(int i = 0; i < NUM_DXL; i++)
    {
      printf("For Servo %d", i);
      dxl_comm_results[i] = packetHandler->write4ByteTxRx(portHandler, DXL_IDS[i], ADDR_GOAL_POSITION, dxl_goal_position[index], &dxl_error);
    
      if (dxl_comm_results[i] != COMM_SUCCESS) {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_results[i]));
      }
      else if (dxl_error != 0) {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      }
    }

    //Move dynamixel
    do {
      for(int i = 0; i < NUM_DXL; i++)
      {
        printf("For Servo %d", i);
        // Read the Present Position
        dxl_comm_results[i] = packetHandler->read4ByteTxRx(portHandler, DXL_IDS[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
        
        if (dxl_comm_results[i] != COMM_SUCCESS) {
          printf("%s\n", packetHandler->getTxRxResult(dxl_comm_results[i]));
        }
        else if (dxl_error != 0) {
          printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }

        printf("[ID:%03d] Goal Position:%03d  Present Position:%03d\n", DXL_IDS[i], dxl_goal_position[index], dxl_present_position);
      }
      
    } while((abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

    // Switch the Goal Position
    if (index == 0) {
      index = 1;
    }
    else {
      index = 0;
    }
  }

  // Disable DYNAMIXEL Torque
  for(int i = 0; i < NUM_DXL; i++)
  {
    printf("For Servo %d", i);
    dxl_comm_results[i] = packetHandler->write1ByteTxRx(portHandler, DXL_IDS[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  
    if (dxl_comm_results[i] != COMM_SUCCESS) {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_results[i]));
    }
    else if (dxl_error != 0) {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    else {
      printf("Succeeded disabling DYNAMIXEL Torque.\n");
    }
  }

  // Close port
  portHandler->closePort();
  return 0;
}
