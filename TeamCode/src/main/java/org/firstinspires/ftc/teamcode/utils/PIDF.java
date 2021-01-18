package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDF
{

    public double kP;
    public double kI;
    public double kD;
    public double f;

    private ElapsedTime timer;
    private double lastError;
    private double target;
    private double I;

    public PIDF(double kP, double kI, double kD, double f)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.f = f;

        timer = new ElapsedTime();
        timer.reset();
        lastError = 0;
        target = 0;
        I = 0;
    }

    public void setTarget(double target)
    {
        this.target = target;
        resetI();
    }

    public double tick(double position)
    {
        double deltaTime = timer.milliseconds();
        double error = target - position;
        double P = kP * error;
        I += kI * error * (deltaTime);
        double D = kD * ((error - lastError) / (deltaTime));
        timer.reset();
        lastError = error;
        return P + I + D + f;
    }

    public void resetI(){
        this.I = 0;
    }

}
