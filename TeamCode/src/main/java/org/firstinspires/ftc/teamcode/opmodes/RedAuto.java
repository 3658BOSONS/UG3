package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Bucket;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.hardware.Wobble;
import org.firstinspires.ftc.teamcode.paths.Red0;
import org.firstinspires.ftc.teamcode.paths.Red1;
import org.firstinspires.ftc.teamcode.utils.AutoAimer;
import org.firstinspires.ftc.teamcode.utils.BulkReadHandler;
import org.firstinspires.ftc.teamcode.utils.MovementPoint;
import org.firstinspires.ftc.teamcode.utils.PIDF;
import org.firstinspires.ftc.teamcode.utils.Positions;
import org.firstinspires.ftc.teamcode.utils.PurePursuit;
import org.firstinspires.ftc.teamcode.utils.RingDetector;
import org.firstinspires.ftc.teamcode.utils.RobotMovement;

import java.util.ArrayList;

@Autonomous(name="RED", group="auto")
public class RedAuto extends LinearOpMode
{

    private Drivetrain dt;
    private Wobble wobble;
    private Shooter shooter;
    private Bucket bucket;
    private Intake intake;

    private AutoAimer aim;
    RingDetector detector;

    private BulkReadHandler bulk;

    private ElapsedTime timer;

    private boolean powerShots;
    private int cPath;
    private int lPath;
    private int lPoint;
    PurePursuit powerDrive;

    private Red0 noring;
    private Red1 onering;

    @Override
    public void runOpMode()
    {
        dt = new Drivetrain(this);
        wobble = new Wobble(true, this);
        shooter = new Shooter(this);
        bucket = new Bucket(this);
        intake = new Intake(this);

        aim = new AutoAimer(dt, shooter);

        //detector = new RingDetector(this);

        bulk = new BulkReadHandler(this);

        timer = new ElapsedTime();
        noring = new Red0();
        onering = new Red1();

        powerShots = false;
        cPath = 0;
        lPath = 0;
        lPoint = 0;

        bucket.setHolding();
        bucket.deactivateOrange();
        dt.getPosition().x = 230;
        dt.getPosition().y = 990;
        shooter.setAngle(22);
        shooter.setTurretAngle(0);
        shooter.setAngleGround();

        ArrayList<MovementPoint> psDrive = new ArrayList<>();
        psDrive.add(new MovementPoint(450, 1000, 0));
        psDrive.add(new MovementPoint(550, 1500, 0));
        psDrive.add(new MovementPoint(700, 1400, Math.toRadians(0), 100));

        powerDrive = new PurePursuit(psDrive);

        waitForStart();

        //READ DISCS
        //int rings = detector.getDecision();
        //detector.stopStreaming();


        int rings = 1; //OVERRIDE FOR EASY EARLY TESTING
        while(opModeIsActive())
        {
            telemetry.clear();
            telemetry.addData("HZ ", bulk.tick(true, false));
            dt.track();

            if(!powerShots){
               tickPowerShots();
            }
            else{
                if(rings == 0)
                {
                    tick0();
                }
                else if(rings == 1)
                {
                    tick1();
                }
            }
            telemetry.update();
        }
    }

    private void tickPowerShots(){
        if(cPath == 0){
            if(powerDrive.purePursuit(dt, this, 1, 1, 0)){
                cPath++;
            }
        }
        else{
            timer.reset();
            bucket.setShooting();
            shooter.spinUp();
            while(opModeIsActive() && timer.milliseconds() < 1500){
                bulk.tick(true, false);
                dt.track();
                double[] drive = aim.track(Positions.psRightX, Positions.psRightY, 900, 1300, 0, 0, 0);
                dt.drive(drive[0], drive[1], drive[2]);
            }
            timer.reset();
            while(opModeIsActive() && timer.milliseconds() < 300){
                bulk.tick(true, false);
                dt.track();
                bucket.index(true);
                double[] drive = aim.track(Positions.psRightX, Positions.psRightY, 900, 1300, 0, 0, 0);
                dt.drive(drive[0], drive[1], drive[2]);
            }

            while(opModeIsActive() && timer.milliseconds() < 1000){
                bulk.tick(true, false);
                dt.track();
                double[] drive = aim.track(Positions.psMidX, Positions.psMidY, 900, 1300, 0, 0, 0);
                dt.drive(drive[0], drive[1], drive[2]);
            }
            timer.reset();
            while(opModeIsActive() && timer.milliseconds() < 300){
                bulk.tick(true, false);
                dt.track();
                bucket.index(true);
                double[] drive = aim.track(Positions.psMidX, Positions.psMidY, 900, 1300, 0, 0, 0);
                dt.drive(drive[0], drive[1], drive[2]);
            }

            while(opModeIsActive() && timer.milliseconds() < 1000){
                bulk.tick(true, false);
                dt.track();
                double[] drive = aim.track(Positions.psLeftX, Positions.psLeftY, 900, 1300, 0, 0, 0);
                dt.drive(drive[0], drive[1], drive[2]);
            }
            timer.reset();
            while(opModeIsActive() && timer.milliseconds() < 300){
                bulk.tick(true, false);
                dt.track();
                bucket.index(true);
                double[] drive = aim.track(Positions.psLeftX, Positions.psLeftY, 900, 1300, 0, 0, 0);
                dt.drive(drive[0], drive[1], drive[2]);
            }
            powerShots = true;
            bucket.setIntaking();
            shooter.setTurretAngle(0);
            shooter.cutPower();
            cPath = 0;
        }
    }

    public void tick0()
    {
        if(cPath == 0)
        {
            if(noring.getPaths().get(cPath).purePursuit(dt, this, .75, 1, 0)){
                cPath++;
            }
        }

        if(cPath == 1 && lPath == 0)
        {
            wobble.lowerArm();
            timer.reset();
            while(timer.milliseconds() < 1000){
                bulk.tick(true, false);
                dt.track();
            }
            wobble.release();
            timer.reset();
            while(timer.milliseconds() < 100){
                bulk.tick(true, false);
                dt.track();
            }
            timer.reset();
            while(timer.milliseconds() < 250){
                bulk.tick(true, false);
                dt.track();
                dt.drive(-Math.PI/2, 1, 0);
            }
        }
        if(cPath == 1){
            double speed = .6;
            if(noring.getPaths().get(cPath).getCurrentPoint() > 1){
                speed = .4;
            }
            if(noring.getPaths().get(cPath).purePursuit(dt, this, speed, 1, 0)){
                cPath++;
            }
        }
        if(cPath == 2 && lPath == 1){
            wobble.grip();
            timer.reset();
            while(timer.milliseconds() < 200){
                bulk.tick(true, false);
                dt.track();
            }
            wobble.raiseArm();
            timer.reset();
            while(timer.milliseconds() < 100){
                bulk.tick(true, false);
                dt.track();
            }
        }
        if(cPath == 2){
            if(noring.getPaths().get(cPath).purePursuit(dt, this, .75, 1, 0)){
                cPath++;
            }
        }
        if(cPath == 3 && lPath == 2){
            wobble.lowerArm();
            timer.reset();
            while(timer.milliseconds() < 750){
                bulk.tick(true, false);
                dt.track();
            }
            wobble.release();
            timer.reset();
            while(timer.milliseconds() < 150){
                bulk.tick(true, false);
                dt.track();
            }
            timer.reset();
            while(timer.milliseconds() < 250){
                bulk.tick(true, false);
                dt.track();
                dt.drive(-Math.PI/2, 1, 0);
            }
            RobotMovement.setPoint(new MovementPoint(1700, 1100, Math.toRadians(45), 50), 1, 1);
            while(!RobotMovement.driveTowardPoint(dt, telemetry, true, 0) && opModeIsActive()){
                bulk.tick(true, false);
                dt.track();
            }
            dt.drive(0, 0, 0);
            while(opModeIsActive()){
                bulk.tick(true, false);
                dt.track();
            }
        }

        lPath = cPath;
        lPoint = noring.getPaths().get(cPath).getCurrentPoint();
    }

    public void tick1()
    {
        if(cPath == 0)
        {
            if(onering.getPaths().get(cPath).purePursuit(dt, this, .75, 1, 0)){
                cPath++;
            }
        }

        if(cPath == 1 && lPath == 0)
        {
            wobble.lowerArm();
            timer.reset();
            while(timer.milliseconds() < 1000){
                bulk.tick(true, false);
                dt.track();
            }
            wobble.release();
            timer.reset();
            while(timer.milliseconds() < 100){
                bulk.tick(true, false);
                dt.track();
            }
            timer.reset();
            while(timer.milliseconds() < 250){
                bulk.tick(true, false);
                dt.track();
                dt.drive(-Math.PI/2, 1, 0);
            }
        }
        if(cPath == 1){
            double speed = .7;
            if(onering.getPaths().get(cPath).getCurrentPoint() > 1){
                speed = .3;
            }
            if(onering.getPaths().get(cPath).purePursuit(dt, this, speed, 1, 0)){
                cPath++;
            }
        }
        if(cPath == 2 && lPath == 1){
            wobble.grip();
            timer.reset();
            while(timer.milliseconds() < 200){
                bulk.tick(true, false);
                dt.track();
            }
            wobble.raiseArm();
            timer.reset();
            while(timer.milliseconds() < 100){
                bulk.tick(true, false);
                dt.track();
            }
            intake.setPower(1);
        }
        if(cPath == 2){
            if(onering.getPaths().get(cPath).purePursuit(dt, this, .75, 1, 0)){
                cPath++;
            }
        }
        if(cPath == 3 && lPath == 2){
            timer.reset();
            while(timer.milliseconds() < 1000){
                bulk.tick(true, false);
                dt.track();
            }
            intake.setPower(0);
            bucket.setShooting();
            shooter.spinUp();
            timer.reset();
            while(timer.milliseconds() < 1000){
                bulk.tick(true, false);
                dt.track();
                double[] powers = aim.track(Positions.goalX, Positions.goalY, 1600, 1000, 0, 0, 24.3);
                dt.drive(powers[0], powers[1], powers[2]);
            }
            timer.reset();
            while(timer.milliseconds() < 300){
                bucket.index(true);
                bulk.tick(true, false);
                dt.track();
            }
            shooter.cutPower();
            bucket.setIntaking();
        }
        if(cPath == 3){
            if(onering.getPaths().get(cPath).purePursuit(dt, this, .75, 1, 0)){
                cPath++;
            }
        }
        if(cPath == 4 && lPath == 3){
            wobble.lowerArm();
            timer.reset();
            while(timer.milliseconds() < 750){
                bulk.tick(true, false);
                dt.track();
            }
            wobble.release();
            timer.reset();
            while(timer.milliseconds() < 150){
                bulk.tick(true, false);
                dt.track();
            }
            timer.reset();
            while(timer.milliseconds() < 250){
                bulk.tick(true, false);
                dt.track();
                dt.drive(-Math.PI/2, 1, 0);
            }
            RobotMovement.setPoint(new MovementPoint(1700, 1100, Math.toRadians(45), 50), 1, 1);
            while(!RobotMovement.driveTowardPoint(dt, telemetry, true, 0) && opModeIsActive()){
                bulk.tick(true, false);
                dt.track();
            }
            dt.drive(0, 0, 0);
            while(opModeIsActive()){
                bulk.tick(true, false);
                dt.track();
            }
        }

        lPath = cPath;
        lPoint = onering.getPaths().get(cPath).getCurrentPoint();
    }

}
