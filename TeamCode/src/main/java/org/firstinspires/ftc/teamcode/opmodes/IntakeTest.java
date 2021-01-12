package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="jintake test", group = "tests")
public class IntakeTest extends LinearOpMode {


    DcMotor intake;
    DcMotor intake2;

    @Override
    public void runOpMode(){
        intake = hardwareMap.get(DcMotor.class, "e1");
        intake2 = hardwareMap.get(DcMotor.class, "e2");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive())
        {
            intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            intake2.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        }
    }
}