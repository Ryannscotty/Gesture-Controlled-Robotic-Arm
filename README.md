# Gesture-Controlled-Robotic-Arm



The goal of this project is to make a gesture controlled robotic arm via wireless grove. I am going to split the work into four main stages,
 3D printing The robot arm, wiring the circuitry in the arm, wireless glove sensors, doing analog to digital signal processing, and coding the input/output ports to do what you want it to do according to the processed signal - which would then translate into actuations of the arm according to what’s connected to each output port.
 Due to signal processing being underdeveloped with no clear way to measure body signal as it is
very low in voltage and the field doesn’t have too much research backing it, meaning this project would still be a
somewhat fresh idea. This is a very important project as many people suffer from motor disabilities
from a stroke or neurological impairments; and these people have a very hard time navigating through life
without one. Additionally, as this project will encompass not just the codework and the circuitry but also
the physical building of the arm, the entire project could in theory be repeatable from start to finish by
someone else when it is neatly documented.

 Introduction
 
 We are developing a means to control a robotic arm via noninvasive wearable sensor circuits, we will
accomplish this in real world applications. The sensor circuits we will be using are a flex sensor and IMU
sensor for our application. This processed signal will be A/D converted and used to create control systems
to actuate the robot arm with gestures from wireless glove. The sensor circuit consists of essentially a
wearable glove with flex sensors and an IMU sensor embedded within the glove as well. This circuit will
use the flex sensors embedded into each finger position to measure the change in resistance as the fingers
begin to move, and our IMU sensor will be placed on the top of the hand to measure the velocity and
angle that the wrist can turn. The combination of these specific two sensors (flex & IMU) can allow
movement for the robot arm and has possible applications for stroke victims with limited motor functions
to control impaired limbs.This project has a huge significance and impact on several different levels, not only is the research
into how sensors can be best adapted to help the human well-being, but this project also has the potential
to apply these robots in situations where it might not be safe for a human to enter. The benefit of further
research into wearable sensors and signal processing in general is a great industry for the medical uses
and benefits from being able to detect better reading from the human body to improve motor functionality
and wellbeing of life and within the body. Having a wearable device that can control a robot for situations
that humans cannot enter is another benefit from this field of study, imagine being able to help rehabilitate
people that suffer from mobility issues or using a robot to help with natural and man-made disasters.


Approach/Design


![Screen Shot 2023-05-28 at 1 13 04 PM](https://github.com/Ryannscotty/Gesture-Controlled-Robotic-Arm/assets/97707478/b0f2fb17-0106-485d-a62c-d743689a5103)


Block Diagram of System Architecture

Mechanical Design

To create a working robotic arm, a well-thought-out mechanical system with multiple degrees of
freedom is necessary. Mechanical design takes into consideration, among many other things, the sorts of
forces that are present in the system and how joints operate. A 3D printer, motors, bearings, and screws
are used to manufacture the whole design of a robotic arm.Our approach for is to design all the necessary parts in Solidworks and have them 3D printed using PLA as our material. The design is divided into three parts: base, arm and gripper. It is divided so that it can be easier to demonstrate and visualize how the Robotic Arm can be assembled.
The first section is the base of the Robotic Arm. The base consists of 8 parts that are assembled to form the base. The 8 parts are: rotating plate, rotating plate clamp, driving gear, cylinder top, cylinder bottom, cylinder middle, bearing ring, and bearing clamp. The base is the part responsible for having the 360-degree rotation around the y-axis. This is achieved by having the rotating plate which will have teeth inside be driven by the gear that will be attached to the DC motor. The figure below you can see a 3D assembled design of the base part. The second section of the Robotic Arm is the arm section. The arm consists of 11 parts: motor mount, left lower arm, right lower arm, center arm, left upper arm, right upper arm, servo mount, bearing holder, bearing guide, lower arm bushing and bearing holder. This section of the Robotic arm is in control of moving up and down as well as being able to extend to reach further. The arm will have left and right motion along the x-axis. The torque required for the motors to move joint can be found using this equation: τ = rFsin(θ) The radius of the arms are 25 cm long. The total load that the DC motors have carry is the weight of the parts the motors that are attached along the arm and the gripper. The are four servo motors that each weight 55g. The parts mass can be found on the solidworks file. The amount of load that will be carried by the motor from 3D printed parts is 1000g. The total load will be 1220g including the four servo motors. An addition 380g will be added into considerations of the robotic arm pick additional objects. Using all the information gathered we can now calculate how much torque the DC motors need in order to be able to run without any difficulties. The torque needed is 5.5N.m (56kg-cm) with a safety factor of 1.5. Since we have 2 DC motors for the joint movement, the total toque can be divided into 2 which is 23kg-cm. The means we need our motor to have 23kg-cm rated torque to perform with out any setback or issues. The last section of the Robotic Arm is the gripper. The gripper consists of 8 parts; bearing bushing(center), bearing guide, bearing bushing (top and bottom), cover plate, gripper (72 teeth), gripper (120 teeth), gear, and mount. The function of this sections is to be able to have strong enough grip to hold and pick up objects such as half full water battle, pen, and ball toys. The gripper will be attached to the arm
section using a servo motor as you can see in the Figure 2 above. This also helps the gripper to have rotation across the x-axis.


![Screen Shot 2023-05-28 at 1 14 17 PM](https://github.com/Ryannscotty/Gesture-Controlled-Robotic-Arm/assets/97707478/ba6f0a1b-4a33-4321-9688-560659df02b6)


3D View Robot Arm. Final and assembled design of the Robot Arm.

![Screen Shot 2023-05-28 at 1 14 39 PM](https://github.com/Ryannscotty/Gesture-Controlled-Robotic-Arm/assets/97707478/97742947-2f97-40f1-8a44-b9789bf0a8a8)















The Electrical Design of Our Wireless glove will consist of a variety of sensors such as flex sensors,
IMU sensors and communication sensors that all will be processed through an ESP32 microcontroller
unit. We will be using a commercial hardware style glove to apply all the necessary sensors and controller
to drive the robotic arms movements. Three fingers will each have a flex sensor attached to it, starting
with the Thumb following index and middle finger, a breadboard will be attached to the top of the glove
which will contain the communication modules, IMU sensor and other various LEDs for event triggering
indications. All power to the glove system will be supplied by a 5V portable battery pack which will be
regulated by a 3.3/5V voltage regulator which will ensure that each component on the glove system
contains proper operating voltages.



![Screen Shot 2023-05-28 at 1 36 57 PM](https://github.com/Ryannscotty/Gesture-Controlled-Robotic-Arm/assets/97707478/1707d21c-b99e-4b1f-a8c4-6da114aa648b)

Wireless Glove Design

Our fully designed wireless glove, as you can see, we have embedded all the
sensor and electronic circuits within and throughout the entire glove. We have three 2.2-inch flex sensors
located on the Thumb, the index and the middle fingers of the glove. The flex sensors have been soldered
with three lead total per each flex sensor. Each flex sensor contains a VCC of 5 volts, a ground lead and
also an analog signal lead that will be sent to the microcontroller to be read into the ESP32’s ADC
module. We have a small breadboard located on the top side of the glove that contains a 5 volt voltage
regulator, an LM324 Operational amplifier acting as a impedance buffer to ensure the flex sensors are
sending the cleanest signal it can be capable of, also the breadboard contains the IMU sensor used to take
speed and directional measurements and assorted LED’s which are acting as indicator for when a data is
being sent from the glove to the robot arm. There are two HC-05 Bluetooth modules located on the
breadboard that each Bluetooth module will control sending data from the flex sensor and IMU sensor
simultaneously and the power of the glove system will be used by a 5-volt output and 2.2-amp current
draw portable battery pack or can be tethered to an AC cable for testing purposes or for unlimited
operating life of the system.





Our approach for the wireless glove will begin with the use and functions of the flex sensors that will
be embedded in the glove. The glove will have three flex sensors located on the thumb, index and middle
finger. The flex sensor is about 2.2 inches long and works as a resistance band potentiometer and as you
bend from flat,45 degrees and 90 degrees voltage level increases. Knowing this sensor contains an
internal resistance of 10K. As you bend the flex sensor the voltage range across the sensor will increase
and if you release the bend of the sensor the voltage will decrease back to the flex sensors normal
operating voltage, by soldering an additional 33K resistor to the flex sensors positive voltage lead you can
receive the optimal voltage range of the sensor which is about 1.5 volts and the flat resistance for this
sensor is 25K whereas the fully bent resistance of the is 70K. the flex sensor is an analog voltage sensor
which voltage will be converted via ADC of the ESP32 microcontroller and then Each finger, the thumb,
index and middle will control/actuate a function on the robot arm.



![Screen Shot 2023-05-28 at 2 58 53 PM](https://github.com/Ryannscotty/Gesture-Controlled-Robotic-Arm/assets/97707478/3987710a-3d76-48b0-b3cb-1a2a569f9623)

Flex Sensor Picture


The next component contained on the wireless glove will be the MPU 6050 6 Degree of Freedom
accelerometer sensor, this sensor will be located on the top of the glove embedded inside a breadboard.
The IMU sensor has the capability to measure the speed (velocity) and angle (degree) of itself as you
move it about the X, Y and Z axis. This sensor is configured through a I2C serial communication link
which contains a serial clock line (SCL) and serial data line (SDA) and these two communication lines
uses the microcontroller system clock to measure the data needed to record the speed and direction
accurately.


![Screen Shot 2023-05-28 at 3 00 59 PM](https://github.com/Ryannscotty/Gesture-Controlled-Robotic-Arm/assets/97707478/639e8162-6fb4-4322-bd8c-ca2cf8399f57)



MPU 6050 (IMU Sensor)



We now come to the wireless communication aspect of our glove. The signals that have been
processed from the three flex sensors, the thumb, index and middle finger and the processed signal X, Y
and Z axis angle and speed measurements coming from the IMU sensor, can now begin the process of
transmitting the data through our HC-05 Bluetooth module to our robot arm from the use of the UART
peripheral module of the ESP32 microcontroller. The HC-05 Bluetooth module has the capability to
change its baud rate using AT commands, this wireless glove system was configured to a baud rate of
115200 rather than using the default 9600 baud rate because of the faster data transmission rate. The
Bluetooth module was also guaranteed to pair with the robot arm by using another change in the AT
commands to set the wireless glove up as a master device while configuring the robot arm’s Bluetooth
module to a slave mode that will only pair with the specified Master modules IP address


![Screen Shot 2023-05-28 at 3 19 03 PM](https://github.com/Ryannscotty/Gesture-Controlled-Robotic-Arm/assets/97707478/cfe28217-6f1d-45a1-beb5-50dcf062e521)


HC-05 Bluetooth Communication Module






All these sensor components will be processed into our ESP32 microcontroller, The ESP32
microcontroller has many peripheral modules such as UART, I2C, SPI and many more functions that will
provide ample possibilities for almost any project. We will be using the ESP32’s ADC, UART and I2C
modules for the wireless glove design, this microcontroller contains a 12-bit ADC which is perfect for
processing the signals coming from the flex sensors because the high ADC 12-bit sample rate allows for a
more precise measurement of these analog signals data points. The 32’s UART module configuration
contains multiple Serial communication channels which is beneficial to this project because the data from
the IMU sensor and from the flex sensors can be sent separately and simultaneously together which can
eliminate possible timing issues within the microcontroller.


![Screen Shot 2023-05-28 at 3 21 53 PM](https://github.com/Ryannscotty/Gesture-Controlled-Robotic-Arm/assets/97707478/9c655b33-91fa-4425-b419-f439f36eb643)


ESP32 Microcontroller



Software Design


The software design of our system will consist of the programming languages C and C++ with the use
of the Arduino IDE software as well, their will be two microcontrollers in use within this project starting
with the ESP32 that will be located on the wireless glove and also a Arduino Mega that will be the
microcontroller that will control all of the functions of the robot arm and the also the servos and DC
motors that drive to robot system. Both MCU’s are capable of being programmed in C and C++. The
robot arm will be controlled by the flex sensors and IMU sensors, with each bend of a flex sensor there
will be a threshold trigger that if exceeded a function will be actuated on the robot arm and if the IMU
sensor of the glove would happen to tilt into certain direction patterns then another function of the robot
arm will be actuated as well. Initially the Robot arm will have two states of control, there will be a wake
up/start up mode that will have the robot arm into a default resting position and when you use the
pushbutton located next to the base of the robot arm then the arm will go into its next state of control
which will be its driving/running state, once in the driving state a user can begin to flex the glove to
actuate to robot arm system.


![Screen Shot 2023-05-28 at 3 28 40 PM](https://github.com/Ryannscotty/Gesture-Controlled-Robotic-Arm/assets/97707478/2634e11c-dbd5-419e-9278-99954cc561f5)



Here is a preview of the first steps on Actuation and control of the Robot Arm System.






https://github.com/Ryannscotty/Gesture-Controlled-Robotic-Arm/assets/97707478/3535b7b5-6b63-43ef-b557-48b00b144fc0







