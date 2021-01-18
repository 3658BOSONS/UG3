package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.PIDF;

public class Shooter {

    private final double targetVelocity = 7000;
    private final double motorRatio = .6;
    private final double paddleClear = .4;
    private final double paddleFull = .64;

    private double turretTarget;
    private final double countsPerRadian = 1/(8192 * 2 / (2 * Math.PI));
    private PIDF turretPid;

    private Motor motor1;
    private Motor motor2;
    private Servo paddle;
    private CRServo turret1;
    private CRServo turret2;

    private Motor turretEnc;

    public Shooter(LinearOpMode opMode){
        motor1 = new Motor(opMode, "shooter1");
        motor2 = new Motor(opMode, "shooter2");
        motor1.setConstants(false, false, true, false);
        motor2.setConstants(false, false, true, false);
        paddle = opMode.hardwareMap.get(Servo.class, "sangle");
        paddle.setPosition(paddleClear);
        turret1 = opMode.hardwareMap.get(CRServo.class, "turret1");
        turret1.setPower(0);
        turret2 = opMode.hardwareMap.get(CRServo.class, "turret2");
        turret2.setPower(0);

        turret1.setDirection(DcMotorSimple.Direction.REVERSE);
        turret2.setDirection(DcMotorSimple.Direction.REVERSE);

        turretEnc = new Motor(opMode, "fl");

        turretPid = new PIDF(1, 0, 1.5, 0);

        motor1.setPID(400, 50, 10);
        motor2.setPID(400, 50, 10);
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
        double power = turretPid.tick(turretEnc.getPosition() * countsPerRadian);

        if(Math.abs(power) > .2){
            power = (Math.abs(power) / power) * .2;
        }

        if(Math.abs(power) < .001)
            power = 0;

        if(Math.abs(power) < .05 && power != 0)
            power = (Math.abs(power) / power) * .05;

        turret1.setPower(power);
        turret2.setPower(power);
        return power;
    }

    public void setTurretTarget(double target)
    {
        turretTarget = target; //radians

        if(turretTarget > Math.PI)
            turretTarget = Math.PI;
        else if(turretTarget < -Math.PI)
            turretTarget = -Math.PI;

        turretPid.setTarget(turretTarget);
    }

    public double getTurretPosition(){
        return turretEnc.getPosition() * countsPerRadian;
    }
    public double getShooterVelo(){
        return motor1.getVelocity() / motorRatio;
    }

}
