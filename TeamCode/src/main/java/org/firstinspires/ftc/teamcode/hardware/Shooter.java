package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.PIDF;

public class Shooter {

    private final double targetVelocity = 3500;
    private final double targetPS = 3000;
    private final double motorRatio = 1;

    private final double angleDown = 0;
    private final double angleUp = .7;
    private final double minAngle = -10;
    private final double maxAngle = 28;

    private final double holderRelease = .73;
    private final double holderDown = .5;

    private final double indexOpen = .2;
    private final double index1 = .4;
    private final double index2 = .55;
    private final double index3 = .8;

    private Motor motor1;
    private Servo liftL;
    private Servo liftR;
    private Servo holder;
    private Servo indexer;

    public Shooter(LinearOpMode opMode){
        motor1 = new Motor(opMode, "shooter");
        motor1.setConstants(true, false, true, true);
        liftL = opMode.hardwareMap.get(Servo.class, "liftL");
        liftR = opMode.hardwareMap.get(Servo.class, "liftR");
        holder = opMode.hardwareMap.get(Servo.class, "holder");
        indexer = opMode.hardwareMap.get(Servo.class, "indexer");

        setAngle(minAngle);
        setReleased();
        setIndexerPos(0);

        motor1.setPID(125, 10, 1);
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
        motor1.setVelocity(2500);
    }

    public void setHolding(){
        holder.setPosition(holderDown);
    }

    public void setReleased(){
        holder.setPosition(holderRelease);
    }

    public void waitThenRaiseChute(){
        Thread t = new Thread(waitRaise);
        t.start();
    }
    public void extendThenRetract(){
        Thread t = new Thread(autoRetract);
        t.start();
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
        if(angle > maxAngle)
            angle = maxAngle;
        if(angle < minAngle)
            angle = minAngle;

        angle = (angle -minAngle) / (maxAngle - minAngle) * (angleUp-angleDown) + angleDown;
        liftL.setPosition(1 - angle);
        liftR.setPosition(angle);
    }

    private Runnable waitRaise = new Runnable(){
        @Override
        public void run(){
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            while(timer.milliseconds()<300){

            }
            setAngle(22);
        }
    };

    private Runnable autoRetract = new Runnable(){
        @Override
        public void run(){
            ElapsedTime timer = new ElapsedTime();
            setIndexerPos(3);
            timer.reset();
            while(timer.milliseconds()<750){

            }
            setIndexerPos(0);
        }
    };

}

