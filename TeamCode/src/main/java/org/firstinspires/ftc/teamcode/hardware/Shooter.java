package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {

    private final double targetSpeed = -.9;
    private final double paddleClear = .4;
    private final double paddleFull = .64;

    private double turretTarget;
    private final double countsPerRadian = 1/(8192 * 2 / (2 * Math.PI));

    private Motor motor1;
    private Motor motor2;
    private Servo paddle;
    private Servo turret1;
    private Servo turret2;

    private Motor turretEnc;

    public Shooter(LinearOpMode opMode){
        motor1 = new Motor(opMode, "shooter1");
        motor2 = new Motor(opMode, "shooter2");
        motor1.setConstants(true, false, true, true);
        motor2.setConstants(true, false, true, true);
        paddle = opMode.hardwareMap.get(Servo.class, "sangle");
        paddle.setPosition(paddleClear);
        turret1 = opMode.hardwareMap.get(Servo.class, "turret1");
        turret1.setPosition(.5);
        turret2 = opMode.hardwareMap.get(Servo.class, "turret2");
        turret2.setPosition(.5);

        turretEnc = new Motor(opMode, "fl");

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

    public void tickTurret()
    {
        double power = ((turretEnc.getPosition() * countsPerRadian) - turretTarget) * .75;
        if(power > 1) power = 1;
        if(power < -1) power = -1;
        turret1.setPosition((power + 1) / 2);
        turret2.setPosition((power + 1) / 2);
    }

    public void setTurretTarget(double target)
    {
        turretTarget = target; //radians

        if(turretTarget > Math.PI)
            turretTarget = Math.PI;
        else if(turretTarget < -Math.PI)
            turretTarget = -Math.PI;
    }

    public double getTurretPosition(){
        return turretEnc.getPosition() * countsPerRadian;
    }
    public double getShooterVelo(){
        return motor2.getVelocity();
    }

}
