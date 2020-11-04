#include "controlledmotor.h"


ControlledMotor::ControlledMotor(DcMotor *_pMotor, Encoder *_pEncoder)
    : pMotor(_pMotor)
    , pEncoder(_pEncoder)
    , Kp(1.0)
    , Ki(0.1)
    , Kd(0.0)
{
    pPID = new PID(&currentSpeed, &output, &setpoint,
                   Kp, Ki, Kd, P_ON_E, DIRECT);
    msSampleTime = 100;
    pPID->SetSampleTime(msSampleTime);
    pPID->SetOutputLimits(-255.0, 255.0);
    pPID->SetMode(AUTOMATIC);
}


void
ControlledMotor::Update() {
    currentSpeed = double(pEncoder->readAndReset())*1000.0/double(msSampleTime);
    pPID->Compute();
    // Update new speed
    pMotor->setSpeed(output);
}


void
ControlledMotor::setTargetSpeed(double newSpeed) {
    setpoint = newSpeed;
}
