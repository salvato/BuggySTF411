#include "controlledmotor.h"


ControlledMotor::ControlledMotor(DcMotor *_pMotor, Encoder *_pEncoder, PID* _pPID)
    : pMotor(_pMotor)
    , pEncoder(_pEncoder)
    , pPID(_pPID)
{
}
