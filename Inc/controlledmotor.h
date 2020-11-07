#pragma once

#include "dcmotor.h"
#include "encoder.h"
#include "PID_v1.h"


class ControlledMotor
{
public:
    ControlledMotor(DcMotor* _pMotor, Encoder* _pEncoder, uint32_t samplingFrequency);
    void Update();
    void setTargetSpeed(double newSpeed);
    void Stop();
    int32_t getTotalMove();

public:
    double   currentSpeed;

private:
    DcMotor* pMotor;
    Encoder* pEncoder;
    PID*     pPID;

private:
    double output;
    double setpoint;
    double sampleTime;
    double Kp;
    double Ki;
    double Kd;
};
