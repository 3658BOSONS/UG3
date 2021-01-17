package org.firstinspires.ftc.teamcode.utils;

public class Pose
{

    public double x;
    public double y;
    public double heading;

    public Pose(double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose(){
        x = 0;
        y = 0;
        heading = 0;
    }

    public void setPose(double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

}
