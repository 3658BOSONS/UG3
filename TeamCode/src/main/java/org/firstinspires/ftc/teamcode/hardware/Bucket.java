package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Bucket {

    private final double magDown = .35;
    private final double mag1 = .43;
    private final double mag2 = .53;
    private final double magUp = .645;
    private final double rotDown = .7;
    private final double rotUp = .52;
    private final double rotMiddle = .6;
    private final double indexIn = .65;
    private final double indexOut = .5;

    private Servo orange;
    private Servo indexer;
    private Servo rotate;
    private Servo mag;

    private boolean isIndexing;
    private boolean isReleasing;
    private int shootingLevel;
    private boolean isRaising;
    private ElapsedTime resetTimer;

    private final int resetTime = 80;
    private final int pushTime = 80;
    private final int raiseTime = 300;

    public Bucket(LinearOpMode opMode){
        orange = opMode.hardwareMap.get(Servo.class, "borange");
        indexer = opMode.hardwareMap.get(Servo.class, "bindexer");
        rotate = opMode.hardwareMap.get(Servo.class, "brotate");
        mag = opMode.hardwareMap.get(Servo.class, "bmag");

        orange.setPosition(.5);
        indexer.setPosition(indexIn);
        rotate.setPosition(rotDown);
        mag.setPosition(magDown);
        shootingLevel = 0;

        isIndexing = false;
        isReleasing= false;
        isRaising = false;
        resetTimer = new ElapsedTime();
        resetTimer.reset();
    }

    public void setIntaking(){
        mag.setPosition(magDown);
        shootingLevel = 0;
        Thread t = new Thread(lowerBucket);
        t.start();
    }

    public void resetIndexing(){
        isIndexing = false;
        isReleasing = false;
        isRaising = false;
    }

    public void setHolding()
    {
        rotate.setPosition(rotMiddle);
    }

    public void setShooting(){
        Thread t= new Thread(raiseBucket);
        t.start();
    }

    public void unJamBucket()
    {
        Thread t = new Thread(jitter);
        t.start();
    }

    public void unJamMag()
    {
        Thread t = new Thread(resetMag);
        t.start();
    }

    public void deactivateOrange(){
        orange.setPosition(.5);
    }

    public void reverseOrange(boolean reversed){
        if(reversed)
            orange.setPosition(0);
        else
            orange.setPosition(1);
    }

    public boolean index(boolean buttonPressed)
    {
        if(buttonPressed && !isIndexing && !isReleasing && !isRaising){
            indexer.setPosition(indexOut);
            isIndexing = true;
            resetTimer.reset();
        }

        if(isIndexing && resetTimer.milliseconds() >= pushTime){
            isIndexing = false;
            resetTimer.reset();
            indexer.setPosition(indexIn);
            isReleasing = true;
        }

        if(isReleasing && resetTimer.milliseconds() >= resetTime){
            isReleasing = false;
            /*if(shootingLevel == 0){
                shootingLevel = 1;
                mag.setPosition(mag2);
            }else if(shootingLevel == 1){
                shootingLevel = 2;
                //mag.setPosition(magUp);
                unJamMag();
            }*/
            resetTimer.reset();
            isRaising = true;
            return true;
        }

        if(isRaising && resetTimer.milliseconds() >= raiseTime){
            isRaising = false;
        }
        return false;
    }

    private Runnable lowerBucket = new Runnable()
    {
        @Override
        public void run()
        {
            long start = System.currentTimeMillis();
            while(System.currentTimeMillis() - start < 250);
            indexer.setPosition(indexIn);
            rotate.setPosition(rotDown);
            orange.setPosition(1);
        }
    };

    private Runnable raiseBucket = new Runnable()
    {
        @Override
        public void run()
        {
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            rotate.setPosition(rotUp);
            while(timer.milliseconds() < 200);
            mag.setPosition(magUp);
            indexer.setPosition(indexIn);
            orange.setPosition(.5);
        }
    };

    private Runnable jitter = new Runnable()
    {
        @Override
        public void run()
        {
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            rotate.setPosition(rotUp);
            while(timer.milliseconds() < 100);
            rotate.setPosition(rotMiddle);
        }
    };

    private Runnable resetMag = new Runnable()
    {
        @Override
        public void run()
        {
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            mag.setPosition(magDown);
            while(timer.milliseconds() < 75);
            mag.setPosition(magUp);
        }
    };
}
