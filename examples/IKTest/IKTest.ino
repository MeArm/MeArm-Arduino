/* MeArm IK test - York Hackspace May 2014
 * Test applying Nick Moriarty's Inverse Kinematics solver
 * to MeArm Robotics Classic MeArm robot arm, to walk a specified path.
 *
 * Pins:
 * Arduino    Base   Shoulder  Elbow    Claw
 *    GND    Brown     Brown   Brown     Brown
 *     5V      Red       Red     Red       Red
 *     11    Yellow
 *     10             Yellow
 *      9                     Yellow
 *      6                               Yellow
 */
#include "MeArm.h"
#include <Servo.h>

int basePin = 11;
int shoulderPin = 10;
int elbowPin = 9;
int clawPin = 6;

/*
//ESP8266 MeArm Wifi PCB
int basePin = 16;
int shoulderPin = 14;
int elbowPin = 12;
int clawPin = 13;
*/

MeArm arm;

void setup() {
  arm.begin(basePin, shoulderPin, elbowPin, clawPin);
}

void loop() {
  //Clap - so it's obvious we're at this part of the routine
  arm.openClaw();
  arm.closeClaw();
  arm.openClaw();
  arm.closeClaw();
  arm.openClaw();
  delay(500);
  //Go up and left to grab something
  arm.moveToXYZ(-80,100,140); 
  arm.closeClaw();
  //Go down, forward and right to drop it
  arm.moveToXYZ(70,200,10);
  arm.openClaw();
  //Back to start position
  arm.moveToXYZ(0,100,50);
  delay(2000);
}
