package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Wobble {

    private final double down = .15;
    private final double aboveWall = .7;
    private final double init = .95;
    private final double gripping = .45;
    private final double open = .16;

    private Servo gripper;
    private Servo rotate;

    public Wobble(boolean isAuto, LinearOpMode opMode){
        gripper = opMode.hardwareMap.get(Servo.class, "wgripper");
        rotate = opMode.hardwareMap.get(Servo.class, "warm");

        if(isAuto){
            gripper.setPosition(gripping);
            rotate.setPosition(init);
        }else{
            gripper.setPosition(open);
            rotate.setPosition(aboveWall);
        }
    }

    public void grip(){
        gripper.setPosition(gripping);
    }
    public void release(){
        gripper.setPosition(open);
    }
    public void lowerArm(){
        rotate.setPosition(down);
    }
    public void raiseArm(){
        rotate.setPosition(aboveWall);
    }
    public void lowerALittle(){
        rotate.setPosition(.35);
    }

}
