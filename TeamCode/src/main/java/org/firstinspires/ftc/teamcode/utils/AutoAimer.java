package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Shooter;

public class AutoAimer
{

    private Drivetrain dt;
    private Shooter shooter;

    private PIDF translation;
    private PIDF heading;

    public AutoAimer(Drivetrain dt, Shooter shooter){
        this.dt = dt;
        this.shooter = shooter;

        translation = new PIDF(.01, 0,.5, 0);
        heading = new PIDF(3.25, 0, .25, 0);
    }

    public double[] track(double targetX, double targetY, double posX, double posY, double angleOffset, double headingOffset, double angleIntercept){
        double[] dtValues = new double[3];

        double xToGoal = targetX - dt.getPosition().x;
        double yToGoal = targetY - dt.getPosition().y;
        double distanceToGoal = Math.hypot(xToGoal, yToGoal);
        double angleToGoal = Math.atan2(yToGoal, xToGoal);
        double dtAngle = BosonMath.clipAngle(dt.getPosition().heading);
        shooter.setTurretAngle(angleToGoal - dtAngle + headingOffset);

        if(Math.abs(angleToGoal - dtAngle + headingOffset) > Math.toRadians(50)){
            heading.setTarget(angleToGoal - dtAngle + headingOffset);
            dtValues[2] = heading.tick(dtAngle);
        }

        translation.setTarget(Math.hypot(posX, posY));
        dtValues[1] = Math.abs(translation.tick(Math.hypot(dt.getPosition().x, dt.getPosition().y)));
        double xToSpot = posX - dt.getPosition().x;
        double yToSpot = posY - dt.getPosition().y;
        double angleToSpot = Math.atan2(yToSpot, xToSpot);
        dtValues[0] = angleToSpot + Math.PI/2 - dt.getPosition().heading;

        shooter.setAngle((1.0/380.0)*distanceToGoal + angleIntercept + angleOffset);

        return dtValues;
    }

}
