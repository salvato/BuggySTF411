#include "controlledmotor.h"


ControlledMotor::ControlledMotor(DcMotor *_pMotor, Encoder *_pEncoder, uint32_t samplingFrequency)
    : pMotor(_pMotor)
    , pEncoder(_pEncoder)
    , sampleTime(1.0/double(samplingFrequency))// in sec.
    , Kp(48.0)
    , Ki(4.0)
    , Kd(0.0)
{
    output       = 0.0;
    currentSpeed = 0.0;
    setpoint     = 0.0;
    pPID = new PID(&currentSpeed, &output, &setpoint,
                   Kp, Ki, Kd, P_ON_E, DIRECT);
    int msSampleTime = int(1000.0*sampleTime);

    pPID->SetSampleTime(msSampleTime);
    pPID->SetOutputLimits(-255.0, 255.0);
    pPID->SetMode(AUTOMATIC);
}


void
ControlledMotor::Update() {
    currentSpeed = pEncoder->readAndReset()/sampleTime; // in Giri/sec
    pPID->Compute();
    // Update new speed
    pMotor->setSpeed(output);
    //pMotor->setSpeed(64.0);
}


void
ControlledMotor::setTargetSpeed(double newSpeed) {
    setpoint = newSpeed;
}
