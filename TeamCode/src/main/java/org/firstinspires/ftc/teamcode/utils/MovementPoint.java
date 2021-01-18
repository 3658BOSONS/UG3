package org.firstinspires.ftc.teamcode.utils;

public class MovementPoint
{

    private double x;
    private double y;
    private double theta; //between 0 and 2pi
    private double headingX;
    private double headingY;
    private double tolerance;

    public MovementPoint(double x, double y, double theta, double tolerance)
    {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.tolerance = tolerance;
    }

    public MovementPoint(double x, double y, double tolerance)
    {
        this.x = x;
        this.y = y;
        this.theta = -5; //arbitrary number outside of range
        this.tolerance = tolerance;
    }

    public MovementPoint(double x, double y, double theta, double tolerance, double headingX, double headingY)
    {
        this.x = x;
        this.y = y;
        this.theta = -5; //arbitrary number outside of range
        this.tolerance = tolerance;
        this.headingX = headingX;
        this.headingY = headingY;
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public double getTheta(){
        return theta;
    }

    public double getTolerance(){
        return tolerance;
    }

    public double getHeadingX(){
        return headingX;
    }

    public double getHeadingY(){
        return headingY;
    }

}
