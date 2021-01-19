package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.PIDF;

public class Shooter {

    private final double targetVelocity = 8000;
    private final double motorRatio = .5;
    private final double paddleClear = .4;
    private final double paddleFull = .64;

    private double CPR = 8192 * 2 / (2 * Math.PI);

    private Motor motor1;
    private Motor motor2;
    private Servo paddle;
    private Servo turret1;
    private Servo turret2;

    private Motor turretEnc;
    private double turretTarget;
    private double turretValue;

    public Shooter(LinearOpMode opMode){
        motor1 = new Motor(opMode, "shooter1");
        motor2 = new Motor(opMode, "shooter2");
        motor1.setConstants(true, false, true, false);
        motor2.setConstants(true, false, true, false);
        paddle = opMode.hardwareMap.get(Servo.class, "sangle");
        paddle.setPosition(paddleClear);
        turret1 = opMode.hardwareMap.get(Servo.class, "turret1");
        turret1.setPosition(.5);
        turret2 = opMode.hardwareMap.get(Servo.class, "turret2");
        turret2.setPosition(.5);

        turretEnc = new Motor(opMode, "fl");
        turretTarget = 0;

        motor1.setPID(50, 10, 5);
        motor2.setPID(50, 10, 5);
    }

    public void spinUp(){
        motor1.setVelocity(targetVelocity * motorRatio);
        motor2.setVelocity(targetVelocity * motorRatio);
    }

    public void cutPower(){
        motor1.setVelocity(0);
        motor2.setVelocity(0);
    }

    public void setAngle(double angle){
        angle = angle - 22;
        double angleDistance = 23;
        double distance = paddleFull - paddleClear;
        paddle.setPosition((angle/angleDistance)*distance + paddleClear);
    }

    public void setTurretAngle(double angle){
        if(angle > Math.toRadians(75)){
            angle = Math.toRadians(75);
        }else if(angle < Math.toRadians(-75)){
            angle = Math.toRadians( -75);
        }

        angle += Math.toRadians(75);
        turret1.setPosition(angle/Math.toRadians(150) - .03);
        turret2.setPosition(angle / Math.toRadians(150) - .02);
    }

    public double getShooterVelo(){
        return motor2.getVelocity() / motorRatio;
    }

}
