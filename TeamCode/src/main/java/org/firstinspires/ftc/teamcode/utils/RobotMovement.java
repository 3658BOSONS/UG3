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
    public static PIDF xyPID = new PIDF(.1, 0, .5, 0);
    private static PIDF headingPID = new PIDF(2, 0, 1, 0);

    public static void setPoint(MovementPoint point, double maxS, double maxT){
        targetPoint = point;
        maxSpeed = maxS;
        maxTurn = maxT;
        lastTTheta = 0;

        xyPID.setTarget(0.0);
        headingPID.setTarget(0.0);
    }

    public static boolean driveTowardPoint(Drivetrain dt, Telemetry telemetry, boolean isLastPoint, int reverse) //Sets the DT power to move towards the point, returns true when within tolerance
    { //1 for reverse, 0 for not
        double dX = targetPoint.getX() - dt.getPosition().x;
        double dY = targetPoint.getY() - dt.getPosition().y;
        double cTheta = BosonMath.clipAngle(dt.getPosition().heading);
        double tTheta = targetPoint.getTheta();
        if(tTheta == -5){
            tTheta = BosonMath.clipAngle(Math.atan2(targetPoint.getHeadingY() - dt.getPosition().y, targetPoint.getHeadingX() - dt.getPosition().x) + (Math.PI * reverse));
        }

        double distToPoint = Math.hypot(dX, dY);
        double dirToPoint =  Math.atan2(dY, dX) - cTheta + Math.PI/2;

        if(Math.abs(distToPoint) < 100 && targetPoint.getTheta() == -5){ //If within 50mm of target, stop changing the heading
            tTheta = lastTTheta;
        }

        if(Math.abs(tTheta - cTheta) > Math.PI){
            tTheta = (Math.toRadians(360) - Math.abs(tTheta)) * (Math.abs(cTheta) / cTheta);
        }

        xyPID.setTarget(Math.hypot(targetPoint.getX(), targetPoint.getY()));
        headingPID.setTarget(tTheta);

        double driveSpeed = xyPID.tick(Math.hypot(dt.getPosition().x, dt.getPosition().y));
        double turnSpeed = headingPID.tick(cTheta);

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

        dt.drive(dirToPoint, Math.abs(driveSpeed), turnSpeed);
        lastTTheta = tTheta;
        return false;
    }
}
