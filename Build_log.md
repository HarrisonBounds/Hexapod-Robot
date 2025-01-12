# Hexapod Build Log

- For this project, I opted to use the dynamixel motors and a Nvidia Jetson Orin Nano to control them. Dynamixel motors can be controlled by daisy chaining, allowing for easy wiring and (hopefully) programming.

<br>

- Before I got started programming the dynamixels, I wanted to build the CAD model for the first leg. To start with the model, I imported the dynamixel XL-430-250 T as well as the connectors that are also available from the same manufacturer. One connector is to have a joint connection on both sides of the servo for stability, and the other is to easily attach something to the servo such as the base, tibia, or femur.

<div style="text-align: center;">
    <img src="media/dynamixel_xl_430.jpg" alt="Dynamixel" width="500" height="300">
</div>


<img src="media/FR11-H101KSet.png" alt="Hexapod Joint Diagram" width="500" height="300">

<img src="media/dynamixel_xl_430.jpg" alt="Hexapod Joint Diagram" width="500" height="300">




<br>

- Below is a diagram of the joints beloinging to the hexapod, and this is how I will refer to them throughout this log:

<div style="text-align: center;">
    <img src="media/hexapod_joint_diagram.jpg" alt="Hexapod Joint Diagram" width="300" height="300">
</div>




