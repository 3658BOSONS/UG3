package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private Motor intake;
    private Servo deploy;

    private final double deployIn = .1;
    private final double deployOut = .9;

    public Intake(LinearOpMode opMode){
        intake = new Motor(opMode, "intake");
        deploy = opMode.hardwareMap.get(Servo.class, "deploy");
        deploy.setPosition(deployIn);
        intake.setConstants(false, false, true, true);
    }

    public void deployIntake(){
        deploy.setPosition(deployOut);
    }

    public void setPower(double power){
        intake.setPower(power);
    }

}
