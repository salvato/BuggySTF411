#pragma once

#include "dcmotor.h"
#include "encoder.h"
#include "PID_v1.h"


class ControlledMotor
{
public:
    ControlledMotor(DcMotor* _pMotor, Encoder* _pEncoder);
    void Update();
    void setTargetSpeed(double newSpeed);

public:
    PID*     pPID;
    double   speed;
    double   output;
    double   setpoint;

private:
    DcMotor* pMotor;
    Encoder* pEncoder;

private:
    int    msSampleTime;
    double Kp;
    double Ki;
    double Kd;
};
