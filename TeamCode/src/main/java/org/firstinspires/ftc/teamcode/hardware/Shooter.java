package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {

    private final double targetSpeed = -.35;
    private final double paddleClear = .4;
    private final double paddleFull = .64;

    private Motor motor1;
    private Motor motor2;
    private Servo paddle;
    private Servo turret1;

    public Shooter(LinearOpMode opMode){
        motor1 = new Motor(opMode, "s1");
        motor2 = new Motor(opMode, "s2");
        motor1.setConstants(true, false, true, true);
        motor2.setConstants(true, false, true, true);
        paddle = opMode.hardwareMap.get(Servo.class, "spaddle");
        paddle.setPosition(.4); //EYOOOOV
        turret1 = opMode.hardwareMap.get(Servo.class, "turret1");
        turret1.setPosition(.56);

        motor1.setPID(500, 0, 0);
        motor2.setPID(500, 0, 0);
    }

    public void spinUp(){
        motor1.setPower(targetSpeed);
        motor2.setPower(targetSpeed);
    }

    public void cutPower(){
        motor1.setPower(0);
        motor2.setPower(0);
    }

    public void setAngle(double angle){
        angle = angle - 22;
        double angleDistance = 23;
        double distance = paddleFull - paddleClear;
        paddle.setPosition((angle/angleDistance)*distance + paddleClear);
    }

}
