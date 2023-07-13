MeArm Arduino
==============

Inverse kinematics control library for MeArm Robotics MeArm with Arduino.

The MeArm has four mini servos - one for the claw, and one each to rotate the base, shoulder joint and elbow joint.  But it's not terribly convenient to be specifying things in terms of servo angles when you're much more interested in where you would like to place the claw, in normal Cartesian (x, y, z) coordinates.

This library solves the angles required to send to the servos in order to meet a given position, allowing for much simpler coding.

Coordinates are measured in mm from the base rotation centre.  Initial 'home' position is at (0, 100, 50), i.e. 100mm forward of the base and 50mm off the ground.

Various other versions of this library exist:
* [Arduino with Adafruit PWM driver board](https://github.com/RorschachUK/meArm_Adafruit)
* [Raspberry Pi with Adafruit PWM driver board](https://github.com/RorschachUK/meArmPi)
* [Beaglebone Black](https://github.com/RorschachUK/meArmBBB)

[![MeArm moving with Inverse Kinematics](http://img.youtube.com/vi/HbxhVs3UmuE/0.jpg)](http://www.youtube.com/watch?v=HbxhVs3UmuE)

Usage
-----

```
#include "MeArm.h"
#include <Servo.h>

MeArm arm;

void setup() {
  arm.begin(11, 10, 9, 6);
  arm.openClaw();
}

void loop() {
  //Go up and left to grab something
  arm.moveToXYZ(-80,100,140); 
  arm.closeClaw();
  //Go down, forward and right to drop it
  arm.moveToXYZ(70,200,10);
  arm.openClaw();
  //Back to start position
  arm.moveToXYZ(0,100,50);
}
```

Three usage examples are included:
* IKTest follows a pre-programmed path defined in Cartesian coordinates
* JoystickIK uses two analogue thumb sticks to guide the claw in Cartesian space
* MeArm_Wii_Classic uses a Wii Classic gamepad connected over I2C to guide the claw

Installation
------------
Clone this repository to your local machine, and place it in your Arduino libraries folder as 'MeArm'.

Class methods of MeArm object
-----------------------------
* void begin(int pinBase, int pinShoulder, int pinElbow, int pinClaw) - The four PWM-capable pins used to drive the servos.  Begin must be called in setup() before any other calls to the MeArm instance are made.
* void openClaw() - opens the claw, letting go of anything it was holding
* void closeClaw() - closes the claw, perhaps grabbing and holding something as it does so
* void moveToXYZ(float x, float y, float z) - move in a straight line from the current point to the requested position
* void snapToXYZ(float x, float y, float z) - set the servo angles to immediately go to the requested point without caring what path the arm swings through to get there - faster but less predictable than moveTo
* void moveTo(float theta, float r, float z) - move in a straight line from the current point to the requested position in cylindrical polar coodrinates
* void snapTo(float theta, float r, float z) - set the servo angles to immediately go to the requested point without caring what path the arm swings through to get there - faster but less predictable than moveTo in cylindrical polar coodrinates
* bool isReachable() - returns true if the point can theoretically be reached by the arm
* float getX() - current x coordinate
* float getY() - current y coordinate
* float getZ() - current z coordinate
* void end() - Disable all servo motors. If you want to continue using the MeArm again later, call begin() again.
