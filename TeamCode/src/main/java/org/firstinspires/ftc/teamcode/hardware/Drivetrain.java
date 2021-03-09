package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.Pose;

public class Drivetrain {

    private Motor fl;
    private Motor fr;
    private Motor bl;
    private Motor br;

    private OdoWheel left;
    private  OdoWheel right;
    private OdoWheel back;

    private Pose position = new Pose();

    public static double xOffset = 0;
    public static double yOffset = 0;
    public static double headingOffset = 0;

    private final double REAR_DIST = 100.0;
    private final double TRACK_WIDTH = 371.6; //1797 3 tiles

    public Drivetrain(LinearOpMode opMode)
    {
        fl = new Motor(opMode, "fl");
        fr = new Motor (opMode, "fr");
        bl = new Motor(opMode, "bl");
        br = new Motor(opMode, "br");

        fl.setConstants(false, true, true, true);
        fr.setConstants(false, true, true, false);
        bl.setConstants(false, true, true, true);
        br.setConstants(false, true, true, false);

        left = new OdoWheel(false, 58, fl);
        right = new OdoWheel(false, 58, br);
        back = new OdoWheel(true, 58, fr);
    }

    public void resetEncoder(){
        fl.setConstants(false, true, true, true);
        br.setConstants(false, true, true, false);
        fr.setConstants(false, true, true, false);

        left = new OdoWheel(false, 58, fl);
        right = new OdoWheel(false, 58, br);
        back = new OdoWheel(true, 58, fr);
    }

    public void setOffsets(double x, double y, double heading){
        xOffset = x;
        yOffset = y;
        headingOffset = heading;
    }

    public void drive(double theta, double power, double turn){

        double x = power * Math.cos(theta);
        double y = power * Math.sin(theta);

        double flPower = y + x - turn;
        double blPower = y - x - turn;
        double frPower = y - x + turn;
        double brPower = y + x + turn;

        double maxPower = 1;

        if(Math.abs(flPower) > maxPower || Math.abs(blPower) > maxPower
                || Math.abs(frPower) > maxPower || Math.abs(brPower) > maxPower)
        {
            double max = Math.max(Math.abs(flPower), Math.abs(blPower));
            max = Math.max(max, Math.abs(frPower));
            max = Math.max(max, Math.abs(brPower));

            flPower /= max;
            blPower /= max;
            frPower /= max;
            brPower /= max;
        }

        fl.setPower(flPower);
        bl.setPower(blPower);
        fr.setPower(frPower);
        br.setPower(brPower);

    }

    public void track()
    {
        double aLeft = left.getMM();
        double aRight = right.getMM();
        double dLeft = left.getDeltaMM();
        double dRight = right.getDeltaMM();
        double dBack = back.getDeltaMM();

        double heading = ((aRight - aLeft) / TRACK_WIDTH) + headingOffset;
        double dHeading = heading - position.heading;

        double dY = dBack + (REAR_DIST * dHeading);
        double dX = (dLeft + dRight) / 2;

        double dXA = dX;
        double dYA = dY;

        if(dHeading != 0){
            double sinTerm = Math.sin(dHeading)/dHeading;
            double cosTerm = (Math.cos(dHeading) - 1)/dHeading;

            dXA = sinTerm * dX - cosTerm* dY;
            dYA = cosTerm * dX + sinTerm * dY;
        }

        double r = Math.sqrt((dXA * dXA) + (dYA * dYA));
        double t = Math.atan2(dYA, dXA);
        t += position.heading;
        double xf = r * Math.cos(t);
        double yf = r * Math.sin(t);

        position.x = position.x + xf;
        position.y = position.y + yf;
        position.heading = heading;
    }

    public void trackWithoutOffsets()
    {
        double aLeft = left.getMM();
        double aRight = right.getMM();
        double dLeft = left.getDeltaMM();
        double dRight = right.getDeltaMM();
        double dBack = back.getDeltaMM();

        double heading = ((aRight - aLeft) / TRACK_WIDTH);
        double dHeading = heading - position.heading;

        double dY = dBack + (REAR_DIST * dHeading);
        double dX = (dLeft + dRight) / 2;

        double dXA = dX;
        double dYA = dY;

        if(dHeading != 0){
            double sinTerm = Math.sin(dHeading)/dHeading;
            double cosTerm = (Math.cos(dHeading) - 1)/dHeading;

            dXA = sinTerm * dX - cosTerm* dY;
            dYA = cosTerm * dX + sinTerm * dY;
        }

        double r = Math.sqrt((dXA * dXA) + (dYA * dYA));
        double t = Math.atan2(dYA, dXA);
        t += position.heading;
        double xf = r * Math.cos(t);
        double yf = r * Math.sin(t);

        position.x = position.x + xf;
        position.y = position.y + yf;
        position.heading = heading;
    }

    public Pose getPosition(){
        return position;
    }

}
