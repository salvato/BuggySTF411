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
    double   currentSpeed;

private:
    DcMotor* pMotor;
    Encoder* pEncoder;
    PID*     pPID;

private:
    double output;
    double setpoint;
    int    msSampleTime;
    double Kp;
    double Ki;
    double Kd;
};
