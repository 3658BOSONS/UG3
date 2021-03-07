package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.PIDF;

public class Shooter {

    private final double targetVelocity = 4000;
    private final double targetPS = 5400;
    private final double motorRatio = 1;

    private final double angleDown = 0;
    private final double angleUp = .8;
    private final double minAngle = -10;
    private final double maxAngle = 28;

    private final double holderRelease = .5;
    private final double holderDown = .4;

    private final double indexOpen = .2;
    private final double index1 = .4;
    private final double index2 = .6;
    private final double index3 = .8;

    private Motor motor1;
    private Servo liftL;
    private Servo liftR;
    private Servo holder;
    private Servo indexer;

    public Shooter(LinearOpMode opMode){
        motor1 = new Motor(opMode, "shooter1");
        motor1.setConstants(true, false, true, false);
        liftL = opMode.hardwareMap.get(Servo.class, "liftL");
        liftR = opMode.hardwareMap.get(Servo.class, "liftR");
        holder = opMode.hardwareMap.get(Servo.class, "holder");
        indexer = opMode.hardwareMap.get(Servo.class, "indexer");

        setAngle(minAngle);
        setReleased();
        setIndexerPos(0);

        motor1.setPID(150, 4, 1);
    }

    public void setPid(double P, double I, double D){
        motor1.setPID(P, I, D);
    }

    public void spinUp(){
        motor1.setVelocity(targetVelocity * motorRatio);
    }

    public void spinPowershots(){
        motor1.setVelocity(targetPS * motorRatio);
    }

    public void cutPower(){
        motor1.setVelocity(1500);
    }

    public void setHolding(){
        holder.setPosition(holderDown);
    }

    public void setReleased(){
        holder.setPosition(holderRelease);
    }

    public void setIndexerPos(int pos){
        switch(pos){
            case 1:
                indexer.setPosition(index1);
                break;
            case 2:
                indexer.setPosition(index2);
                break;
            case 3:
                indexer.setPosition(index3);
                break;
            default:
                indexer.setPosition(indexOpen);
                break;
        }
    }

    public double getShooterVelo(){
        return motor1.getVelocity() / motorRatio;
    }

    public void setAngle(double angle){
        angle = (angle -minAngle) / maxAngle * (angleUp-angleDown) + angleDown;
        liftL.setPosition(1 - angle);
        liftR.setPosition(angle);
    }

}
