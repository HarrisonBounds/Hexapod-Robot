# Hexapod Build Log

- For this project, I opted to use the dynamixel motors and a Nvidia Jetson Orin Nano to control them. Dynamixel motors can be controlled by daisy chaining, allowing for easy wiring and (hopefully) programming.

<br>

- Before I got started programming the dynamixels, I wanted to build the CAD model for the first leg. To start with the model, I imported the dynamixel XL-430-250 T as well as the connectors that are also available from the same manufacturer. One connector is to have a joint connection on both sides of the servo for stability, and the other is to easily attach something to the servo such as the base, tibia, or femur.

- Here is a link to the E-manual for the servo I am using in this project
[Link Text](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/)


<div style="text-align: center;">
    <img src="media/dynamixel_xl_430.jpg" alt="Dynamixel" width="300" height="150">



<img src="media/FR11-H101K Set.png" alt="FR11-H101K Set" width="200" height="200">

<img src="media/FR12-S102K Set.png" alt="FR12-S102K Set" width="300" height="200">

</div>


### Week 1
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



https://github.com/user-attachments/assets/1626d32c-aedb-42f3-aecb-51aee07d06fc



### Week 2
- This week I had several plans to optimze version one of my leg, and to get the servos working:

<br>

- Make the femur shorter
- Make the tibia longer
- Put cutouts in both to have a lighted leg
- Make supports so the cutouts wouldn't be too flimsy
- Set up the Dynamixel Wizard to change the servo ids
- Get my servos moving simultaneously 

<br>

I ended up accomplishing all of these goals. Starting with the hardware, my new design looked a lot cooler and more functional. The new femur, tibia, and assembly is below:

<div style="text-align: center;">

<img src="media/femur_v2.png" alt="femur_v2" width="300" height="300">

<img src="media/tibia_v2.png" alt="tibia_v2" width="300" height="300">

</div>


<div style="text-align: center;">
    <img src="media/assembly_v2.png" alt="assembly_v2" width="500" height="400">
</div>

<br>

- While this was printing, I wanted to test my servos to get them moving at the same time. I used the dynamixel wizard to change each servo unique ids and update all firmware. 


- I also edited the read_write code to move multiple servos at once. Once the new leg is printed I can start programming the inverse kinematics for one leg.


https://github.com/user-attachments/assets/9b11daee-581e-4daf-9013-fdc824fcda0e












