# Hexapod Build Log

- For this project, I opted to use the dynamixel motors and a Nvidia Jetson Orin Nano to control them. Dynamixel motors can be controlled by daisy chaining, allowing for easy wiring and (hopefully) programming.

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

<br>
<br>

- After getting the leg assembles, I was ready to try to program the dynamixel. I started by cloning the Dynamixel SDK and running their example file "read_write.cpp". Although I should have went through this code before (and ran it on a dynamixel not attached to anything), I went ahead and ran it. It worked! 

- Unfortunately, the code had the servo rotating 360 degrees, completely breaking my 3d printed bracket.

- Finally I imported that code into my project and edited the Makefile to run it from my personal directory. The dynamixel is moving!!

https://github.com/user-attachments/assets/1626d32c-aedb-42f3-aecb-51aee07d06fc



### Week 2
- This week I had several plans to optimze version one of my leg, and to get the servos working:

<br>

1. Make the femur shorter
2. Make the tibia longer
3. Put cutouts in both to have a lighted leg
4. Make supports so the cutouts wouldn't be too flimsy
5. Set up the Dynamixel Wizard to change the servo ids
6. Get my servos moving simultaneously 

<br>

- I ended up accomplishing all of these goals. Starting with the hardware, my new design looked a lot cooler and more functional. The new femur, tibia, and assembly is below:

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

- After making the servos move in sync, I put my new leg together and clamped it on the side of the table so I could start writing the Inverse Kinematic Equations. 

<div style="text-align: center;">
    <img src="media/hexapod_leg_v2.png" alt="leg_v2" width="500" height="400">
</div>

- Now it is time to solve some inverse kinematics! I have one joint that rotates on the x-y plane and two joints that rotate on the y-z plane. Lets start with the first joint.

<div style="text-align: center;">
    <img src="media/ik_hexapod_leg.png" alt="ik" width="400" height="400">
</div>

1. Find the angle $\theta_1$ by looking at a top down view on the leg (since this joint moves on the x-y plane aka back and forth). Calculating this joint is not so bad since its joint one jont on the plane, meaning we can make use of the angle tan($\theta_1$) = $\frac{opp}{hyp}$ or tan($\theta_1$) = $\frac{x_{offset}}{y_{offset}}$. Rearanging this equation, we have for our first joint: 

<center>

$\theta_1 = \tan^{-1}\left(\frac{x}{y}\right)$

</center>

2. $\theta_3$ is the next easiest to find, as all we need to do is calculate the top angle of the red triangle above. For this we can implement the law of cosines:

<center>

$L^2 = R2^2 + R3^2 - 2R2R3 \cdot \cos(\theta_3)$

Rearranging this to solve for $\theta_3$ we have:

$\theta_3 = \cos^{-1}\left(\frac{R2^2 + R3^2 - L^2}{2R2R3}\right)$


</center>

3. Now that we have $\theta_1$ and $\theta_3$, we can solve for $\theta_2$. To get $\theta_2$, we need to first get $\phi_2$ and $\phi_1$. Once we get these values, we can subtract them to getting the reamining angle $\theta_2$.

4. $\phi_2$ - Despite how this looks, this is NOT a right triangle. This means we will one again have to deploy the law of cosines to get our answer. 

<center>

$\phi_2 = \cos^{-1}\left(\frac{R2^2 + L^2 - R3^2}{2R2L}\right)$

</center>

5. $\phi_1$ - Since you can make a rectangle or right angle out of the blue triangle, we can say that $\phi_1$ is equivalent to the interior angle at the end of the blue triangle. With this clever trick in mind, we can solve for $\phi_1$:

<center>

$\phi_1 = \tan^{-1}\left(\frac{z_{offset}}{y_{offset}}\right)$

</center>

6. Finally we can get $\theta_2$! NOTE: Since the z offset is negative, our $\phi_1$ also becomes a negative. To mitigate this, we now will add both $\phi_1$ and $\phi_2$ together to get a postivie angle.

7. Also, for the sake of simplicity I kept y<sub>offset</sub> in the equation. But actually, y<sub>offset</sub> is too long. Instead, in the code I calculate the distance between the first joint and the foot, then subtract the length between J1 and J2. This gives me the actual distance.

8. I struggled for a long time trying to figure out why hte equations weren't working. I was positive that the calculations were correct. Well it turns out these angles expect every joint to be set at a 0 angle. So in the code I first set the leg to its rest position (pass in xyz = 0), and then I calculate a position from there. In the future I will probably set all of my servos to the same joint position, so I can go to the home position easier.

<br>

- And with that, we have the inverse kinematics laid out for our leg! The next step was to test this out in code. I wrote a script for it by editing the original read_write.cpp file, but to no surprise it was more difficult to intergrate all together. While I had a break I decided to start designing the base of my robot. 

- I decided to make the base a hexagon (hexapod -> hexagon), which I think makes sense. Since I was know thinking about a base, I needed to have enough room to attach all of my electronics. 

- I had the Nvidia Jetson Orin Nano, U2D2 controller, and the yet was to be detemined. What power source should I use? How would I successfully daisy chain all of the servos in a clean way?

- I ended up getting this board from Trossen Robotics for the daisy chaining issue. Now keep in mind, I have no idea the amperage limits on this board, much less if it actually works yet. Thats on my to do list. The board is called 6 Port XM/XL Power Hub (3pin). 

<div style="text-align: center;">
    <img src="media/Trossen_power_hub.png" alt="leg_v2" width="300" height="300">
</div>

- My next issue was the power supply. I needed a power supply for my Jetson (7~20V, 5A). I also needed a power supply for this board above (which I still have no idea if it works yet). According to my servos the operating voltage is 11.1V and the stall current is 1.3 amps. 

- For now, I wont worry about that, I will just keep my robot tethered to test everything.

### Week 3

- This week I designed the base, and got everything put together like it should. I also got the inverse kinematics working completely for one leg.

- One issue I have had is telling the leg to move 50mm in the z direction. After moving there, moving it back down -50mm goes further that it was originally. Soon I will experiment giving it smaller waypoints, so hopefully there will be less room for error.

<div style="text-align: center;">
    <img src="media/complete_hexapod.png" alt="leg_v2" width="500" height="400">
</div>

https://github.com/user-attachments/assets/047964a4-c2c0-47c0-82e6-51405fc5a215


- This week my controller came in, so soon I will start trying to read joystick input, so I can eventually control the hexapod with it

- Currently, I am trying to print all of my parts. Each leg takes ~7 hours to print, and the base will most likely take about the same or a little less. After these parts are all printed, I can assemble the robot and do more in depth testing with a more stable connection.

### Week 4

- The Inverse Kinematics is finally working completely for one leg! This was my main hiccup that I used all of Week 3 to solve. 

- While putting together my legs via the 3d printed parts, my brackets started to break. This lead me to reinforce these brackets in CAD to make them much stronger and more stable

- The first iteration of my base has been designed, and I will be using acrylic to test it (because laser cutting is much faster than printing). 

- I want to wait to use my controller until I have most of the robot assembled until I start testing joystick input and event controls

https://github.com/user-attachments/assets/4bb8eb8e-8e12-4749-ad72-5f7c3612408d

<div style="text-align: center;">
    <img src="media/servo_bracket_new.png" alt="servo_bracket_new" width="200" height="200">
    <img src="media/joint_bracket_new.png" alt="joint_bracket_new" width="300" height="200">
</div>

- My first iteration of my base cut through after several passes on the laser, but I realized it was way to big for my robot. I decided that the next iteration should be smaller, and also circular for ease of design.

<div style="text-align: center;">
    <img src="media/first_iteration_base.png" alt="first_iteration_base" width="250" height="300">
</div>



### Week 5
- I created a wider base to hold all of my electronics, including the jetson and batteries on the top, and the U2D2 Power hubs and voltage regulators on the bottom.

- I found that this method extends the robot too far out creating additional torque that the motorss need to provide. These week was mainly constructing and printing this base while trying to get it standing. 

<div style="text-align: center;">
    <img src="media/LargeBase.png" alt="first_iteration_base" width="250" height="300">
</div>

- I now have several iterations of CAD designs with different componenets. All of these are visible on my CAD page and allows to to go back to a design if I need to. 

- I made the brackets much thicker so they are more sturdy, but the servo brackets still have a thin hole where the screw sits flush to the mount. This connection is not too good, since there is not much material to go in between. Soon I want to print out more of these, as they have become an issue during testing

### Week 6

- With my new base, I got the robot standing, but the feet slide out from under it pretty easily. To mitigate this, I ordered some rubber feet to stick on the bottom of the feet.

- Although the robots stand, the base is still too big. I have designed a new base that is not circular, so that I can put all of my components in line without having to extend the base material in all directions. I believe this will save me some weight. 

- Also to save weight, I bought a new small voltage regualtor that is rated for 20 amps, and a lipo splitter to split the batteries current to both of my power hubs. With the other battery gone, and both of my heavy voltage regualtors off board, this reduces my weight by ~0.75 kg or 1.7 lbs. This is going to be really nice for less torue requirements on the motors

- Although it pained me, I decided to take out the "femur" portion of my leg. Reducing this length helps provide more stability and less torque for the motors. 

- I wanted the robot to go to the standing position from any leg position. I made my code modular, making a Leg structure so I can control each leg with its own values. This has proven more diffcult than I thought, as I haven't necessarily gotten it to fully work yet.

<div style="text-align: center;">
    <img src="media/Standing.png" alt="first_iteration_base" width="350" height="300">
</div>

<div style="text-align: center;">
    <img src="media/Maintenance.png" alt="first_iteration_base" width="350" height="300">
</div>

### Week 7

The robot stands! Everything is mounted onto the base and the run can run untethered. The issue here is that the feet need to slide under the robot for it to support itself, so the floor needs to not have mucch friction.

However, when testing the walking, I noticed there was something wrong with the gait. I had accidentally programmed the turning gait instead. 

![Turning1](https://github.com/user-attachments/assets/54235994-e868-42e8-8928-91541ae943e0)


I thought it was an easy fix but instead of that, I had to dive into the mechanics of the legs to see how each one was moving in respect to its tripod. This took some time because I kepy having to reprogram and test. I did end up getting this dancing motion where the feet weren't moving at all. This was my favorite mistake Ive made.

![Uploading DANCE.gif…]()


### Week 8 

Finally I have the hexapod moving!! My first goal is all but done. The hexapod returns to home every it takes a step, which is not ideal. It's not too big of a deal because the tripod gait works well. The robot moves in a straight line and looks good while doing it! 

![Tripod_example_short](https://github.com/user-attachments/assets/4613c453-170d-4409-8f91-8c828e82dfdf)


I wanted a way to take the base off of the top without having to unscrew it everytime. I thought magnets could work well if mounted with standoffs, but after time "wasted" with this, I figured it would be best to move on to my stretch goal. 

I had tried multiple times before to export my URDF from onshape using a tool called onshape-to-robot. To no surprise there was more to it than it seemed. A cohort mate of mine, Han, helped me specify all of the correct requirements, and I finally remade my CAD model for the millionth time and got it to visualize in pybullet. Pybullet was just a test to see if the urdf would actually move with the correct joint locations, and at the end of this week it did!!

### Week 9

The Reinforcement Learning part of this project seems daunting, but luckily I was working on another project training the Unitree Go2 robot dog to perform different tasks. I used some code from the other project, added two new legs (both the hexapod and the dog have three joints per leg), and the training script was running! 

![hexapod_rl_short](https://github.com/user-attachments/assets/5cd3d814-59f4-4c74-9089-af690775fa50)


Its walking! Currently its fairly slow, after getting it working, I needed to decide if I wanted to tune it to be perfect, or start working on the sim-to-real. I decided to pursue the sim-to-real option, although time was running out and I had many other deadlines to handle.

At the end of this week I added a menu for the hexapod. This way instad of a controller (too much for what im trying to do), the user can just give it commands from a simple menu. This menu includes walking, turning, and waving right now. Before showing this off at the MSI museum in a few weeks, I want to make this menu more robust and include more options, but for right now this is what we got. 

### Week 10

Final Week!! I can't believe I made it this far. I built the hexapod from scratch, programmed it to walk untethered with a tripod gait, turn, wave, and applied reinforcement learning in simulation! I really wanted to achieve sim-to-real, but I think its best to continue the progress I have made into the next quarter. 

My thoughts with sim-to-real is to clip the actions outputted by the policy network and apply them to the joints of the hexapod (low level control). This and some tuning will hopefully give me some insight on next steps, and help me research how to make sim-to-real possible. 




















