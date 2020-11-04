#include "controlledmotor.h"


ControlledMotor::ControlledMotor(DcMotor *_pMotor, Encoder *_pEncoder)
    : pMotor(_pMotor)
    , pEncoder(_pEncoder)
    , Kp(12.0)
    , Ki(4.0)
    , Kd(0.0)
{
    pPID = new PID(&currentSpeed, &output, &setpoint,
                   Kp, Ki, Kd, P_ON_E, DIRECT);
    int msSampleTime = 100;
    sampleTime = 1000.0/msSampleTime; // in sec
    pPID->SetSampleTime(msSampleTime);
    pPID->SetOutputLimits(-255.0, 255.0);
    pPID->SetMode(AUTOMATIC);
}


void
ControlledMotor::Update() {
    currentSpeed = double(pEncoder->readAndReset())*sampleTime;
    pPID->Compute();
    // Update new speed
    pMotor->setSpeed(output);
}


void
ControlledMotor::setTargetSpeed(double newSpeed) {
    setpoint = newSpeed;
}
