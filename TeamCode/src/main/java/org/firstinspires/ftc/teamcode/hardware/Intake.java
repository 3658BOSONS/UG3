package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Intake {

    private Motor front;
    private Motor in;

    public Intake(LinearOpMode opMode){
        front = new Motor(opMode, "if");
        in = new Motor(opMode, "ii");

        front.setConstants(false, false, true, true);
        in.setConstants(false, false, true, true);
    }

    public void setPower(double power){
        front.setPower(power);
        in.setPower(power);
    }

}
