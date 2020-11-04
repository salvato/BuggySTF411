#include "controlledmotor.h"


ControlledMotor::ControlledMotor(DcMotor *_pMotor, Encoder *_pEncoder)
    : pMotor(_pMotor)
    , pEncoder(_pEncoder)
{
    pPID = new PID(&speed, &output, &setpoint,
                   Kp, Ki, Kd, P_ON_E, DIRECT);
}
