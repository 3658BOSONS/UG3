package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Drivetrain {

    private Motor fl;
    private Motor fr;
    private Motor bl;
    private Motor br;

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
    }

    public void drive(double theta, double power, double turn){

        double x = power * Math.cos(theta) * 1.5;
        double y = power * Math.sin(theta);

        double flPower = y + x + turn;
        double blPower = y - x + turn;
        double frPower = y - x - turn;
        double brPower = y + x - turn;

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

}
