
#include "motorController.h"
#include "dcmotor.h"
#include "encoder.h"
#include "PID_v1.h"


MotorController::MotorController(DcMotor* motor, Encoder* tachometer)
    : pMotor(motor)
    , pTachometer(tachometer)
{
    speedMax    = 8.0;// In giri/s
    targetSpeed = 0.0;

    currentP = 0.0;
    currentI = 0.0;
    currentD = 0.0;
    pPid = new PID(currentP, currentI, currentD, DIRECT);

    msSamplingTime = 100;
    pPid->SetSampleTime(msSamplingTime);
    pPid->SetMode(AUTOMATIC);
    pPid->SetOutputLimits(-255.0, 255.0);

    bTerminate = false;
}


void
MotorController::setPIDmode(int Mode) {
    pPid->SetMode(Mode);
}


void
MotorController::go() {
}


void
MotorController::updateSpeed() {
    if(!bTerminate) {
        currentSpeed = pTachometer->read()/speedMax;
        double commandedSpeed = pPid->Compute();
        pMotor->setSpeed(commandedSpeed);
    }
    else {
        pMotor->stop();
    }
}


void
MotorController::setP(double p) {
    currentP = p;
    pPid->SetTunings(currentP, currentI, currentD);
}


void
MotorController::setI(double i) {
    currentI = i;
    pPid->SetTunings(currentP, currentI, currentD);
}


void
MotorController::setD(double d) {
    currentD = d;
    pPid->SetTunings(currentP, currentI, currentD);
}


void
MotorController::setSpeed(double speed) {
    targetSpeed = speed;
}


void
MotorController::terminate() {
    bTerminate = true;
}
