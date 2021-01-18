package org.firstinspires.ftc.teamcode.utils;

public class BosonMath
{

    public static double clipAngle(double angle){
        double y = Math.sin(angle);
        double x = Math.cos(angle);
        return Math.atan2(y, x);
    }

}
