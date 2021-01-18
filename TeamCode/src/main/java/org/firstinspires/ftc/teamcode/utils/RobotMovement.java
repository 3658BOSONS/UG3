package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;

public class RobotMovement
{
    //wes if you are reading this I assume you are programming possibly late at night if so i ask this one question
//Should you be up this late?
//go to bed nerd
    private static MovementPoint targetPoint;
    private static double maxSpeed;
    private static double maxTurn;
    private static double lastTTheta;
    public static PIDF xyPID = new PIDF(.003, 0, .000, 0);
    private static PIDF headingPID = new PIDF(.8, 0, 0, 0);

    public static void setPoint(MovementPoint point, double maxS, double maxT){
        targetPoint = point;
        maxSpeed = maxS;
        maxTurn = maxT;
        lastTTheta = 0;

        xyPID.setTarget(0.0);
        headingPID.setTarget(0.0);
    }

    public static boolean driveTowardPoint(Drivetrain dt, State state, Telemetry telemetry, boolean isLastPoint, int reverse) //Sets the DT power to move towards the point, returns true when within tolerance
    { //1 for reverse, 0 for not
        double dX = targetPoint.getX() - dt.getPosition().x;
        double dY = targetPoint.getY() - dt.getPosition().y;
        double cTheta = Math.atan2(Math.sin(dt.getPosition().heading), Math.cos(dt.getPosition().heading)); //-pi, pi
        double tTheta = targetPoint.getTheta();
        if(tTheta == -5){
            tTheta = Math.atan2(dY, dX) + (Math.PI * reverse);
            tTheta = Math.atan2(Math.sin(tTheta), Math.cos(tTheta));
        }

        double distToPoint = Math.sqrt((dX * dX) + (dY * dY));
        double dirToPoint =  Math.atan2(dY, dX) - cTheta - (Math.PI / 2);

        if(Math.abs(distToPoint) < 200 && targetPoint.getTheta() == -5){ //If within 50mm of target, stop changing the heading
            tTheta = lastTTheta;
        }

        double dTheta = tTheta - cTheta;
        if(Math.abs(dTheta) > Math.PI){
            dTheta = (360 - Math.abs(dTheta)) * -(Math.abs(dTheta) / dTheta);
        }

        double driveSpeed = xyPID.tick(distToPoint);
        double turnSpeed = headingPID.tick(dTheta);

        if(Math.abs(driveSpeed) > maxSpeed || !isLastPoint){
            driveSpeed = maxSpeed * (Math.abs(driveSpeed) / driveSpeed);
        }
        if(Math.abs(turnSpeed) > maxTurn){
            turnSpeed = maxTurn * (Math.abs(turnSpeed) / turnSpeed);
        }

        if(Math.abs(distToPoint) <  targetPoint.getTolerance()){
            dt.drive(0, 0, 0);
            return true;
        }

        telemetry.addData("distance to point", distToPoint);
        telemetry.addData("target", 0);
        telemetry.addData("Drive Speed", driveSpeed);

        dt.drive(driveSpeed, dirToPoint, turnSpeed);
        lastTTheta = tTheta;
        return false;
    }
}
