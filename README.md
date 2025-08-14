# Hexapod Robot Control

This repository contains the C++ code for controlling a hexapod robot equipped with Dynamixel servomotors and Python code for training the same hexapod in a simulated environement. The code implements inverse and forward kinematics, tripod gait for walking, turning maneuvers, and a waving motion. It utilizes the Dynamixel SDK for communication with the motors.

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

---

## Prerequisites

* **Dynamixel SDK:** You need to have the Dynamixel SDK installed on your system. Follow the installation instructions provided in the official [ROBOTIS-MANIPULATOR-H](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/) documentation.
* **Dynamixel Servomotors:** This code is designed for Dynamixel X-series servomotors. The `DXL_IDS` array in the code defines the IDs of the 18 motors used (3 per leg). Ensure your motors are properly connected.
* **USB2DYNAMIXEL:** A USB to Dynamixel communication interface is required to connect your computer to the Dynamixel bus.
* **Serial Port Permissions:** Ensure your user has the necessary permissions to access the serial port specified by `DEVICENAME` (e.g., `/dev/ttyUSB0` on Linux). You might need to add your user to the `dialout` group (Linux).

---

## Hardware Configuration

The code assumes a hexapod robot with the following motor ID configuration:

* **Leg 1:** Motors 1, 2, 3
* **Leg 2:** Motors 4, 5, 6
* **Leg 3:** Motors 7, 8, 9
* **Leg 4:** Motors 10, 11, 12
* **Leg 5:** Motors 13, 14, 15
* **Leg 6:** Motors 16, 17, 18

The `Leg` structure also defines the `home_positions` for each motor in each leg. These values should correspond to the robot's default standing or "home" configuration. You might need to adjust these values based on your robot's physical setup.

---

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
---
## Usage

1.  **Connect the Robot:** Ensure your hexapod robot is powered on and connected to your computer via the USB2DYNAMIXEL adapter.

2.  **Run the Executable:** Execute the compiled program:
    ```bash
    ./hexapod
    ```

3.  **Observe the Output:** The program will initialize the Dynamixel motors, move the robot to its home position, and then continuously print the current position of each leg.

4.  **Interactive Menu (Commented Out):** To enable the interactive menu, uncomment the `displayMenu()` call and the `switch` statement in the `main` function. You can then control the robot's actions by pressing the corresponding numbers and 'ESC' to exit.


---

## Inverse Kinematics
$$
\begin{align}
y &\leftarrow y + Y_{REST} \\
z &\leftarrow z + Z_{REST}
\end{align}
$$

$$
\begin{align}
\theta_1 &= \arctan\left(\frac{x}{y}\right) \cdot \frac{180}{\pi}
\end{align}
$$

$$
\begin{align}
r_1 &= \sqrt{x^2 + y^2} - R_1 \\
r_2 &= z \\
d &= \sqrt{r_1^2 + r_2^2}
\end{align}
$$

$$
\begin{align}
\phi_1 &= \arctan\left(\frac{z}{r_1}\right) \cdot \frac{180}{\pi} \\
\phi_2 &= \arccos\left(\frac{R_2^2 + d^2 - R_3^2}{2 \cdot R_2 \cdot d}\right) \cdot \frac{180}{\pi} \\
\phi_3 &= \arccos\left(\frac{R_2^2 + R_3^2 - d^2}{2 \cdot R_2 \cdot R_3}\right) \cdot \frac{180}{\pi}
\end{align}
$$

$$
\begin{align}
\theta_2 &= \phi_1 + \phi_2 \\
\theta_3 &= \phi_3 - 90
\end{align}
$$

---

## Reinforcement Learning in Genesis

"[Genesis](https://genesis-world.readthedocs.io/en/latest/) is a physics platform designed for general purpose Robotics/Embodied AI/Physical AI applications." 

This hexapod traning adapts the provided genesis RL training for the Unitree go2 robot dog, and applies it to the hexapod. It uses [Proximal Policy Optimization](https://openai.com/index/openai-baselines-ppo/) (PPO) to perform the training. This algorithm is an Actor-Critic method that tries to maximize the future reward by applying actions from the actor to robot in the simualted environment.

The gait comparison done in this project was a programmed tripod gait vs. a learned gait. The learned gait closely resembles a tripod movement for the hexapod, but with more tuning, this can be different for difference application (speed, stability, etc.)

**To Train:**

```
cd hexapod_rl/
python3 -m venv hex_venv
source hex_venv/bin/activate
pip install -r requirements.txt
python3 hexapod_train.py
python3 hexapod_eval.py
```

This will train in headless mode, and show the evaluation visualization at the end of training. This this is a simple locomotion policy, the training should not take more than 10 minutes.



