#pragma once

#include "dcmotor.h"
#include "encoder.h"
#include "PID_v1.h"


class ControlledMotor
{
public:
    ControlledMotor(DcMotor* _pMotor, Encoder* _pEncoder, PID *_pPID);

private:
    DcMotor* pMotor;
    Encoder* pEncoder;
    PID*     pPID;
};
