package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.PIDF;

public class Shooter {

    private final double targetVelocity = 5600;
    private final double targetPS = 5400;
    private final double motorRatio = .66;
    private final double paddleClear = .24;
    private final double paddleFull =-.06;

    private Motor motor1;
    private Motor motor2;
    private Servo paddle;
    private Servo turret1;
    private Servo turret2;

    private Motor turretEnc;
    private double turretTarget;
    private final double CPR = 8192 * 2 / (2 * Math.PI);

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

        motor1.setPID(150, 4, 1);
        motor2.setPID(150, 4, 1);
    }

    public void setAngleGround(){
        paddle.setPosition(0);
    }

    public void setPid(double P, double I, double D){
        motor2.setPID(P, I, D);
        motor1.setPID(P, I, D);
    }

    public void resetTurretEncoder(){
        turretEnc.setConstants(false, true, true, true);
    }

    public void spinUp(){
        motor1.setVelocity(targetVelocity * motorRatio);
        motor2.setVelocity(targetVelocity * motorRatio);
    }

    public void spinPowershots(){
        motor1.setVelocity(targetPS * motorRatio);
        motor2.setVelocity(targetPS * motorRatio);
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
        double maxAngle = 1.08;
        angle -= .08;
        if(angle > maxAngle){
            angle = maxAngle;
        }else if(angle < -maxAngle){
            angle = -maxAngle;
        }

        angle /= maxAngle;
        double pos = angle / (maxAngle);
        pos += 1;
        pos /= 2;

        turretTarget = angle;
        turret1.setPosition(pos);
        turret2.setPosition(pos);
    }

    public double getTurretAngle(){
        return -turretEnc.getPosition() / CPR;
    }

    public double getShooterVelo(){
        return motor2.getVelocity() / motorRatio;
    }

}
