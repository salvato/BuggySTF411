#include "controlledmotor.h"


ControlledMotor::ControlledMotor(DcMotor *_pMotor, Encoder *_pEncoder, uint32_t samplingFrequency)
    : pMotor(_pMotor)
    , pEncoder(_pEncoder)
    , sampleTime(1.0/double(samplingFrequency))// in sec.
    , Kp(60.0)
    , Ki(16.0)
    , Kd(0.0)
{
    output       = 0.0;
    currentSpeed = 0.0;
    setpoint     = 0.0;
    pPID = new PID(&currentSpeed, &output, &setpoint,
                   Kp, Ki, Kd, P_ON_E, DIRECT);

    pPID->SetSampleTime(int(1000.0*sampleTime));
    pPID->SetOutputLimits(-255.0, 255.0);
    pPID->SetMode(AUTOMATIC);
}


void
ControlledMotor::Update() {
    currentSpeed = pEncoder->readAndReset()/sampleTime; // in Giri/sec
    if(pPID->Compute())
        pMotor->setSpeed(output);// Update new speed
}


void
ControlledMotor::setTargetSpeed(double newSpeed) {
    setpoint = newSpeed;
    pPID->SetMode(AUTOMATIC);
}


void
ControlledMotor::Stop() {
    pMotor->stop();
    setpoint = 0.0;
    pPID->SetMode(MANUAL);
}

int32_t
ControlledMotor::getTotalMove() {
    return pEncoder->readTotal();
}
