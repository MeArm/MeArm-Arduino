/* MeArm library based on the work by York Hack Space May 2014
 * Officially co-opted as the go to IK and control library July 2023
 * A simple control library for the MeArm Robotics MeArm Robot Arm
 * Usage:
 *   MeArm arm;
 *   arm.begin(1, 10, 9, 6);
 *   arm.openClaw();
 *   arm.moveToXYZ(-80, 100, 140);
 *   arm.closeClaw();
 *   arm.moveToXYZ(70, 200, 10);
 *   arm.openClaw();
 */
#ifndef MEARM_H
#define MEARM_H

#include <Arduino.h>
#include <Servo.h>

const float pi=3.14159265359;

struct ServoInfo {
    int n_min, n_max;   // PWM 'soft' limits - should be just within range
    float gain;         // PWM per radian
    float zero;         // Theoretical PWM for zero angle
};

class MeArm {
  public:
    //Full constructor uses calibration data, or can just give pins
    MeArm(int sweepMinBase=180, int sweepMaxBase=0, float angleMinBase=-pi/4, float angleMaxBase=pi/4,
      int sweepMinShoulder=118, int sweepMaxShoulder=22, float angleMinShoulder=pi/4, float angleMaxShoulder=3*pi/4,
      int sweepMinElbow=144, int sweepMaxElbow=36, float angleMinElbow=pi/4, float angleMaxElbow=-pi/4,
      int sweepMinClaw=75, int sweepMaxClaw=180, float angleMinClaw=pi/2, float angleMaxClaw=0);
    //required before running
    void begin(int pinBase, int pinShoulder, int pinElbow, int pinClaw);
    void end();
    void moveTo(float theta, float r, float z);     //Travel smoothly from current point to another point in cylindrical polar coordinates
    void moveToXYZ(float x, float y, float z);      //Same as above but for cartesian coodrinates
    void snapTo(float theta, float r, float z);     //Set servos to reach a certain point directly without caring how we get there 
    void snapToXYZ(float x, float y, float z);      //Same as above but for cartesian coodrinates
    void openClaw();                                //Grab something
    void closeClaw();                               //Let it go (don't hold it back anymore)
    bool isReachable(float x, float y, float z);    //Check to see if possible
    float getX(); //Current x, y and z
    float getY();
    float getZ();
    float getR();
    float getTheta();
  private:
    void polarToCartesian(float theta, float r, float& x, float& y);
    float _x, _y, _z;
    float _r, _t;
    Servo _base, _shoulder, _elbow, _claw;
    ServoInfo _svoBase, _svoShoulder, _svoElbow, _svoClaw;
    int _pinBase, _pinShoulder, _pinElbow, _pinClaw;
};

#endif
