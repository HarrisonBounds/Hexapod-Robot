# Hexapod Robot Control

This repository contains the C++ code for controlling a hexapod robot equipped with Dynamixel servomotors. The code implements inverse and forward kinematics, tripod gait for walking, turning maneuvers, and a waving motion. It utilizes the Dynamixel SDK for communication with the motors.

## Features

* **Inverse Kinematics (IK):** Calculates the joint angles required to reach a desired Cartesian coordinate for each leg.
* **Forward Kinematics (FK):** Calculates the Cartesian coordinates of the leg's end effector based on the current joint angles.
* **Tripod Gait:** Implements a stable walking motion by coordinating the movement of alternating tripods of legs.
* **Turning:** Enables the robot to turn by coordinating leg movements.
* **Wave Motion:** A demonstration of coordinated leg movement to produce a wave-like action.
* **Dynamixel SDK Integration:** Uses the Dynamixel SDK 2.0 for robust communication with Dynamixel X-series servomotors.
* **Bezier Curves:** Employs Bezier curves to generate smooth and natural-looking motions for walking and waving.
* **Modular Leg Structure:** Defines a `Leg` structure to manage the motor IDs and home/move positions for each leg.
* **Interactive Menu (Commented Out):** Includes a commented-out interactive menu for controlling different actions (walk, turn, wave, home).
* **Real-time Position Feedback:** Continuously reads and displays the current position of each leg.

## Prerequisites

* **Dynamixel SDK:** You need to have the Dynamixel SDK installed on your system. Follow the installation instructions provided in the official [ROBOTIS-MANIPULATOR-H](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/) documentation.
* **Dynamixel Servomotors:** This code is designed for Dynamixel X-series servomotors. The `DXL_IDS` array in the code defines the IDs of the 18 motors used (3 per leg). Ensure your motors are properly connected.
* **USB2DYNAMIXEL:** A USB to Dynamixel communication interface is required to connect your computer to the Dynamixel bus.
* **Serial Port Permissions:** Ensure your user has the necessary permissions to access the serial port specified by `DEVICENAME` (e.g., `/dev/ttyUSB0` on Linux). You might need to add your user to the `dialout` group (Linux).

## Hardware Configuration

The code assumes a hexapod robot with the following motor ID configuration:

* **Leg 1:** Motors 1, 2, 3
* **Leg 2:** Motors 4, 5, 6
* **Leg 3:** Motors 7, 8, 9
* **Leg 4:** Motors 10, 11, 12
* **Leg 5:** Motors 13, 14, 15
* **Leg 6:** Motors 16, 17, 18

The `Leg` structure also defines the `home_positions` for each motor in each leg. These values should correspond to the robot's default standing or "home" configuration. You might need to adjust these values based on your robot's physical setup.

## Software Setup

1.  **Clone the Repository:**
    ```bash
    git clone <repository_url>
    cd <repository_directory>
    ```

2.  **Install Dynamixel SDK (if not already installed):**
    Follow the instructions in the [Dynamixel SDK documentation](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/). Make sure the C++ library is built and installed correctly on your system.

3.  **Compilation:**
    You will need to compile the `main.cpp` file. The compilation process will depend on your system and how you installed the Dynamixel SDK. You might need to create a `CMakeLists.txt` file or use a direct g++ compilation command, linking against the Dynamixel SDK libraries.

    **Example using g++ (assuming the Dynamixel SDK headers and libraries are in standard locations or you adjust the include and link paths accordingly):**
    ```bash
    g++ -o hexapod main.cpp -I/path/to/dynamixel_sdk/include -L/path/to/dynamixel_sdk/lib -ldynamixel_sdk
    ```
    Replace `/path/to/dynamixel_sdk/include` and `/path/to/dynamixel_sdk/lib` with the actual paths to your Dynamixel SDK installation.

## Usage

1.  **Connect the Robot:** Ensure your hexapod robot is powered on and connected to your computer via the USB2DYNAMIXEL adapter.

2.  **Run the Executable:** Execute the compiled program:
    ```bash
    ./hexapod
    ```

3.  **Observe the Output:** The program will initialize the Dynamixel motors, move the robot to its home position, and then continuously print the current position of each leg.

4.  **Interactive Menu (Commented Out):** To enable the interactive menu, uncomment the `displayMenu()` call and the `switch` statement in the `main` function. You can then control the robot's actions by pressing the corresponding numbers and 'ESC' to exit.

## Code Structure

* **`main.cpp`:** Contains the main program logic, including initialization, control loop, and the commented-out interactive menu.
* **Header Includes:** Includes necessary standard C/C++ libraries and the Dynamixel SDK header.
* **Definitions:** Defines constants for control table addresses, baud rate, protocol version, device name, torque enable/disable values, and kinematic parameters.
* **`Leg` Structure:** Defines a structure to hold motor IDs, home positions, and move positions for each leg.
* **`getch()` and `kbhit()`:** Platform-specific functions for non-blocking keyboard input (used for the commented-out interactive menu).
* **`IK(x, y, z, thetaList[])`:** Implements the inverse kinematics algorithm to calculate joint angles for a given end-effector position.
* **`FK(theta1, theta2, theta3, position[])`:** Implements the forward kinematics algorithm to calculate the end-effector position for given joint angles.
* **`getCurrentLegPosition(...)`:** Reads the current positions of the leg motors and calculates the current Cartesian position of each leg's end effector using forward kinematics.
* **`bezierPoint(p0, p1, p2, t)`:** Calculates a point on a quadratic Bezier curve.
* **`move(...)`:** Sends goal position commands to the specified motors using the Dynamixel GroupSyncWrite for synchronized movement.
* **`calculatePosition(...)`:** Converts desired joint angles (from IK) into Dynamixel motor position values.
* **`tripodGait(...)`:** Implements the tripod gait walking motion using Bezier curves for smooth transitions.
* **`turning(...)`:** Implements the turning motion using coordinated leg movements and Bezier curves.
* **`wave(...)`:** Implements a waving motion for one of the legs using Bezier curves.
* **`displayMenu()`:** (Commented Out) Displays the interactive control menu.
* **`returnToHome(...)`:** Moves all legs back to their defined home positions.
* **`main()`:** The main function that initializes the system, enables torque, moves to the home position, and runs the control loop (including the commented-out menu).

## Future Work (Reinforcement Learning Integration)

This section will be updated with details about the reinforcement learning work done to further enhance the robot's control and capabilities. Stay tuned for information on:

* **RL Environment Setup:** How the hexapod control problem is formulated as a reinforcement learning environment.
* **Reward Functions:** The design of reward functions to guide the learning process for specific tasks (e.g., obstacle avoidance, efficient locomotion).
* **RL Algorithms Used:** The specific reinforcement learning algorithms implemented (e.g., Deep Q-Networks (DQN), Proximal Policy Optimization (PPO)).
* **Integration with Sensor Data:** How sensor data (if any) is incorporated into the RL agent's decision-making process.
* **Learned Behaviors:** Descriptions and demonstrations of the behaviors learned by the RL agent.
* **Instructions for Running RL Experiments:** Steps to set up and run the reinforcement learning training and evaluation scripts.

## License

[Specify your license here, e.g., MIT License]