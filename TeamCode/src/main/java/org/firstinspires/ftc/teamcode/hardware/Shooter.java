package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {

    private final double targetVelocity = 9000;
    private final double motorRatio = .6;
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
        motor1.setConstants(true, false, true, false);
        motor2.setConstants(true, false, true, false);
        paddle = opMode.hardwareMap.get(Servo.class, "sangle");
        paddle.setPosition(paddleClear);
        turret1 = opMode.hardwareMap.get(Servo.class, "turret1");
        turret1.setPosition(.5);
        turret2 = opMode.hardwareMap.get(Servo.class, "turret2");
        turret2.setPosition(.5);

        turretEnc = new Motor(opMode, "fl");

        motor1.setPID(100, 0, 5);
        motor2.setPID(100, 0, 5);
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

    public double tickTurret()
    {
        double power = ((turretEnc.getPosition() * countsPerRadian) - turretTarget) * .5;
        if(power > 1) power = 1;
        if(power < -1) power = -1;

        if(Math.abs(power) < .01)
            power = 0;

        if(Math.abs(power) < .05 && power != 0)
            power = (Math.abs(power) / power) * .05;

        turret1.setPosition((power + 1) / 2);
        turret2.setPosition((power + 1) / 2);
        return (power + 1) / 2;
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
        return motor1.getVelocity() / motorRatio;
    }

}
