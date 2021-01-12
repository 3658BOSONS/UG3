package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="servo tuner", group = "tests")
public class ServoTuner extends LinearOpMode {


    Servo servo;
    private double servoPos;
    private boolean lastD;

    @Override
    public void runOpMode(){
        servo = hardwareMap.get(Servo.class, "servo");
        servoPos = .5;
        lastD = false;

        waitForStart();

        while(opModeIsActive())
        {
            servo.setPosition(servoPos);
            //servoPos += gamepad1.left_stick_y*.01;

            if(gamepad1.dpad_down && !lastD){
                servoPos -= .1;
                lastD = true;
            }else if(gamepad1.dpad_up && !lastD) {
                servoPos += .1;
                lastD = true;
            }else if(gamepad1.dpad_right && !lastD) {
                servoPos += .01;
                lastD = true;
            }else if(gamepad1.dpad_left && !lastD){
                servoPos -= .01;
                lastD = true;
            }

            if(!(gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right)){
                lastD = false;
        }


            telemetry.clear();
            telemetry.addData("pos", servoPos);
            telemetry.update();
        }
    }
}