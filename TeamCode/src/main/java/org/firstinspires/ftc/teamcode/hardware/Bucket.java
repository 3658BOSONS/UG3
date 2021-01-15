package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Bucket {

    private final double magDown = .35;
    private final double magUp = .65;
    private final double rotDown = .7;
    private final double rotUp = .52;
    private final double rotMiddle = .57;
    private final double indexIn = .65;
    private final double indexOut = .5;

    private Servo orange;
    private Servo indexer;
    private Servo rotate;
    private Servo mag;

    private boolean isIndexing;
    private long startIndexTime;
    private boolean isReleasing;
    private long startReleaseTime;

    private final int resetTime = 150;
    private final int pushTime = 50;

    public Bucket(LinearOpMode opMode){
        orange = opMode.hardwareMap.get(Servo.class, "borange");
        indexer = opMode.hardwareMap.get(Servo.class, "bindexer");
        rotate = opMode.hardwareMap.get(Servo.class, "brotate");
        mag = opMode.hardwareMap.get(Servo.class, "bmag");

        orange.setPosition(.5);
        indexer.setPosition(indexIn);
        rotate.setPosition(rotDown);
        mag.setPosition(magDown);

        isIndexing = false;
        startIndexTime = 0;
        isReleasing= false;
        startReleaseTime = 0;
    }

    public void setIntaking(){
        mag.setPosition(magDown);
        Thread t = new Thread(lowerBucket);
        t.start();
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

    public void index(boolean buttonPressed)
    {
        if(buttonPressed && !isIndexing && !isReleasing){
            indexer.setPosition(indexOut);
            isIndexing = true;
            startIndexTime = System.currentTimeMillis();
        }

        if(isIndexing && System.currentTimeMillis() - startIndexTime >= pushTime){
            isIndexing = false;
            indexer.setPosition(indexIn);
            startReleaseTime = System.currentTimeMillis();
            isReleasing = true;
        }

        if(isReleasing && System.currentTimeMillis() - startReleaseTime >= resetTime){
            isReleasing = false;
        }
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
            while(timer.milliseconds() < 150);
            mag.setPosition(magUp);
        }
    };
}
