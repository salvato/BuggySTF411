#pragma once

#include "dcmotor.h"
#include "encoder.h"
#include "PID_v1.h"


class ControlledMotor
{
public:
    ControlledMotor(DcMotor* _pMotor, Encoder* _pEncoder);

private:
    DcMotor* pMotor;
    Encoder* pEncoder;
    PID*     pPID;

private:
    double speed;
    double output;
    double setpoint;
    double Kp;
    double Ki;
    double Kd;
};
