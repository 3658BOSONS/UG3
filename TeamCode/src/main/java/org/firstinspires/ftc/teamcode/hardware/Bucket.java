package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Bucket {

    private final double magDown = .35;
    private final double magUp = .65;
    private final double rotDown = .68;
    private final double rotUp = .53;
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

    private final int resetTime = 400;

    public Bucket(LinearOpMode opMode){
        orange = opMode.hardwareMap.get(Servo.class, "orange");
        indexer = opMode.hardwareMap.get(Servo.class, "indexer");
        rotate = opMode.hardwareMap.get(Servo.class, "brotate");
        mag = opMode.hardwareMap.get(Servo.class, "mag");

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
        t.run();
    }

    public void setShooting(){
        rotate.setPosition(rotUp);
        Thread t= new Thread(raiseBucket);
        t.run();
    }

    public void index(boolean buttonPressed)
    {
        if(buttonPressed && !isIndexing && !isReleasing){
            indexer.setPosition(indexOut);
            isIndexing = true;
            startIndexTime = System.currentTimeMillis();
        }

        if(isIndexing && System.currentTimeMillis() - startIndexTime >= resetTime){
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
            while(System.currentTimeMillis() - start < 100);
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
            long start = System.currentTimeMillis();
            while(System.currentTimeMillis() - start < 800);
            mag.setPosition(magUp);
            indexer.setPosition(indexIn);
            orange.setPosition(.5);
        }
    };
}
