# Hexapod Build Log

- For this project, I opted to use the dynamixel motors and a Nvidia Jetson Orin Nano to control them. Dynamixel motors can be controlled by daisy chaining, allowing for easy wiring and (hopefully) programming.

<br>

- Before I got started programming the dynamixels, I wanted to build the CAD model for the first leg. To start with the model, I imported the dynamixel XL-430-250 T as well as the connectors that are also available from the same manufacturer. One connector is to have a joint connection on both sides of the servo for stability, and the other is to easily attach something to the servo such as the base, tibia, or femur.

<div style="text-align: center;">
    <img src="media/dynamixel_xl_430.jpg" alt="Dynamixel" width="300" height="150">



<img src="media/FR11-H101K Set.png" alt="FR11-H101K Set" width="200" height="200">

<img src="media/FR12-S102K Set.png" alt="FR12-S102K Set" width="300" height="200">

</div>


<br>

- Below is a diagram of the joints beloinging to the hexapod, and this is how I will refer to them throughout this log:

<div style="text-align: center;">
    <img src="media/hexapod_joint_diagram.jpg" alt="Hexapod Joint Diagram" width="300" height="300">
</div>

- After importing all of the models into Onshape's Assembly, I could build the femur and the tibia off of the existing pieces. Here is version one of the first hexapod leg with no base:


<div style="text-align: center;">
    <img src="media/Hexapod_leg_cad_v1.png" alt="Hexapod_led_cad_v1" width="500" height="300">
</div>

- After printing and putting the heated inserts into the femur and tibia, the final result
looked like this:

<div style="text-align: center;">
    <img src="media/Hexapod_leg_v1.jpg" alt="Hexapod_led_v1" width="400" height="400">
</div>

#### Building Materials 
- I will come back to this list later to order, so I thought I would list these down

- To attach the fastening brackets (they dont rotate), I just used the screws that came on the dynamixel. 

- **2.5mm*2.5mm*6mmSpacer (x72):** Fastening brackets I have not decided if I want ot 3d print the spacer or order it.

- **M2*4mm Screws (x132):** Fastening the joint brackets together, and attaching the joint brackets to either side of the servo horn. 

- **M2*8mm Screws (x128):** Attaching the femur and tibia to the joints via heated inserts

- **M3*8mm Screws (x18):** Attaching servo horn to servo motor

- **M2 * 8 * 3.5 Heat Inserts(x96)**: Attaching the brackets to the physcial parts of the femur and tibia, and attaching the entire leg to the base


After getting the leg assembles, I was ready to try to program the dynamixel. I started by cloning the Dynamixel SDK and running their example file "read_write.cpp". Although I should have went through this code before (and ran it on a dynamixel not attached to anything), I went ahead and ran it. It worked! 

Unfortunately, the code had the servo rotating 360 degrees, completely breaking my 3d printed bracket.

Finally I imported that code into my project and edited the Makefile to run it from my personal directory. The dynamixel is moving!!

<video width="640" height="360" controls>
  <source src="media/Dynamixel_first_test.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>










