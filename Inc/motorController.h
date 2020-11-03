#pragma once

class DcMotor;
class Encoder;
class PID;


class MotorController
{

public:
    explicit MotorController(DcMotor* motor, Encoder* tachometer);
    void setPIDmode(int Mode);

public:
    void updateSpeed();
    void terminate();
    void go();
    void setSpeed(double speed);
    void setP(double p);
    void setI(double i);
    void setD(double d);

private:
    DcMotor*  pMotor;
    Encoder*  pTachometer;
    PID*      pPid;

    double targetSpeed;
    double currentSpeed;
    volatile bool bTerminate;
    double currentP, currentI, currentD;
    double speedMax;
    int    msSamplingTime;
};
