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
import org.firstinspires.ftc.teamcode.paths.Red4;
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
    private Red4 fourring;

    @Override
    public void runOpMode()
    {
        dt = new Drivetrain(this);
        wobble = new Wobble(true, this);
        shooter = new Shooter(this);
        bucket = new Bucket(this);
        intake = new Intake(this);

        aim = new AutoAimer(dt, shooter);

        detector = new RingDetector(this);

        bulk = new BulkReadHandler(this);

        dt.setOffsets(0, 0, 0);

        timer = new ElapsedTime();
        noring = new Red0();
        onering = new Red1();
        fourring = new Red4();

        powerShots = false;
        cPath = 0;
        lPath = 0;
        lPoint = 0;

        bucket.setShooting();
        bucket.deactivateOrange();
        dt.getPosition().x = 230;
        dt.getPosition().y = 990;
        dt.resetEncoder();
        shooter.setAngle(22);
        shooter.setTurretAngle(0);
        shooter.setAngleGround();

        ArrayList<MovementPoint> psDrive = new ArrayList<>();
        psDrive.add(new MovementPoint(450, 1000, 0));
        psDrive.add(new MovementPoint(550, 1600, 0));
        psDrive.add(new MovementPoint(1200, 1300, 100));

        powerDrive = new PurePursuit(psDrive);

        waitForStart();

        shooter.resetTurretEncoder();
        int rings = detector.getDecision();
        detector.stopStreaming();

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
                else
                {
                    tick4();
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
            shooter.spinPowershots();
            while(opModeIsActive() && timer.milliseconds() < 2000){
                bulk.tick(true, false);
                dt.track();
                double[] drive = aim.track(Positions.psRightX, Positions.psRightY, 1500, 1300, 0, 0, 0);
                dt.drive(drive[0], drive[1], drive[2]);
            }
            timer.reset();
            while(opModeIsActive() && timer.milliseconds() < 300){
                bulk.tick(true, false);
                dt.track();
                bucket.index(true);
                double[] drive = aim.track(Positions.psRightX, Positions.psRightY, 1500, 1300, 0, 0, 0);
                dt.drive(drive[0], drive[1], drive[2]);
            }

            while(opModeIsActive() && timer.milliseconds() < 1000){
                bulk.tick(true, false);
                dt.track();
                double[] drive = aim.track(Positions.psMidX, Positions.psMidY, 1500, 1300, 0, 0, 0);
                dt.drive(drive[0], drive[1], drive[2]);
            }
            timer.reset();
            while(opModeIsActive() && timer.milliseconds() < 300){
                bulk.tick(true, false);
                dt.track();
                bucket.index(true);
                double[] drive = aim.track(Positions.psMidX, Positions.psMidY, 1500, 1300, 0, 0, 0);
                dt.drive(drive[0], drive[1], drive[2]);
            }

            while(opModeIsActive() && timer.milliseconds() < 1000){
                bulk.tick(true, false);
                dt.track();
                double[] drive = aim.track(Positions.psLeftX, Positions.psLeftY, 1500, 1300, 0, 0, 0);
                dt.drive(drive[0], drive[1], drive[2]);
            }
            timer.reset();
            while(opModeIsActive() && timer.milliseconds() < 300){
                bulk.tick(true, false);
                dt.track();
                bucket.index(true);
                double[] drive = aim.track(Positions.psLeftX, Positions.psLeftY, 1500, 1300, 0, 0, 0);
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
            while(timer.milliseconds() < 400){
                bulk.tick(true, false);
                dt.track();
            }
            wobble.raiseArm();
            timer.reset();
            while(timer.milliseconds() < 100){
                bulk.tick(true, false);
                dt.track();
            }
            wobble.lowerArm();
            timer.reset();
            while(timer.milliseconds() < 500){
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
            while(timer.milliseconds() < 500){
                bulk.tick(true, false);
                dt.track();
                dt.drive(-Math.PI/2, 1, 0);
            }
        }
        if(cPath == 1){
            double speed = .7;
            if(noring.getPaths().get(cPath).getCurrentPoint() > 1){
                speed = .25;
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
            dt.setOffsets(dt.getPosition().x, dt.getPosition().y, dt.getPosition().heading);
            while(opModeIsActive());
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
            while(timer.milliseconds() < 400){
                bulk.tick(true, false);
                dt.track();
            }
            wobble.raiseArm();
            timer.reset();
            while(timer.milliseconds() < 75){
                bulk.tick(true, false);
                dt.track();
            }
            wobble.lowerArm();
            timer.reset();
            while(timer.milliseconds() < 500){
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
            bucket.setHolding();
            shooter.spinUp();
            timer.reset();
            while(timer.milliseconds() < 1000){
                bulk.tick(true, false);
                dt.track();
                double[] powers = aim.track(Positions.goalX, Positions.goalY, 1400, 850, 0, 0, 22.2);
                dt.drive(powers[0], powers[1], powers[2]);
            }
            bucket.setShooting();
            timer.reset();
            while(timer.milliseconds() < 750){
                bulk.tick(true, false);
                dt.track();
                double[] powers = aim.track(Positions.goalX, Positions.goalY, 1400, 850, 0, 0, 22.2);
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
            RobotMovement.setPoint(new MovementPoint(1900, 1100, Math.toRadians(45), 50), 1, 1);
            while(!RobotMovement.driveTowardPoint(dt, telemetry, true, 0) && opModeIsActive()){
                bulk.tick(true, false);
                dt.track();
            }
            dt.drive(0, 0, 0);

            dt.setOffsets(dt.getPosition().x, dt.getPosition().y, dt.getPosition().heading);
            while(opModeIsActive());
        }

        lPath = cPath;
        lPoint = onering.getPaths().get(cPath).getCurrentPoint();
    }

    public void tick4()
    {
        if(cPath == 0)
        {
            if(fourring.getPaths().get(cPath).purePursuit(dt, this, .75, 1, 0)){
                cPath++;
            }
        }

        if(cPath == 1 && lPath == 0)
        {
            wobble.lowerArm();
            timer.reset();
            while(timer.milliseconds() < 400){
                bulk.tick(true, false);
                dt.track();
            }
            wobble.raiseArm();
            timer.reset();
            while(timer.milliseconds() < 75){
                bulk.tick(true, false);
                dt.track();
            }
            wobble.lowerArm();
            timer.reset();
            while(timer.milliseconds() < 500){
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
            intake.setPower(-.3);
        }
        if(cPath == 1){
            double speed = .7;
            if(fourring.getPaths().get(cPath).getCurrentPoint() > 1){
                speed = .3;
            }
            if(fourring.getPaths().get(cPath).purePursuit(dt, this, speed, 1, 0)){
                cPath++;
            }
        }
        if(cPath == 2 && lPath == 1){
            intake.setPower(0);
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
            if(fourring.getPaths().get(cPath).purePursuit(dt, this, .75, 1, 0)){
                cPath++;
            }
        }
        if(cPath == 3 && lPath == 2){
            intake.setPower(1);
        }
        if(cPath == 3){
            if(fourring.getPaths().get(cPath).purePursuit(dt, this, .35, 1, 0)){
                cPath++;
            }
        }
        if(cPath == 4 && lPath == 3){
            timer.reset();
            while(timer.milliseconds() < 500){
                bulk.tick(true, false);
                dt.track();
            }
        }
        if(cPath == 4){
            if(fourring.getPaths().get(cPath).purePursuit(dt, this, .4, 1, 0)){
                cPath++;
            }
        }
        if(cPath == 5 && lPath == 4){
            timer.reset();
            while(timer.milliseconds() < 1000){
                bulk.tick(true, false);
                dt.track();
            }
            intake.setPower(0);
            bucket.setHolding();
            shooter.spinUp();
            bucket.unJamBucket();
            timer.reset();
            while(timer.milliseconds() < 750){
                bulk.tick(true, false);
                dt.track();
                double[] powers = aim.track(Positions.goalX, Positions.goalY, 1400, 850, 0, 0, 21.9);
                dt.drive(powers[0], powers[1], powers[2]);
            }
            bucket.setShooting();
            timer.reset();
            wobble.lowerALittle();
            while(timer.milliseconds() < 750){
                bulk.tick(true, false);
                dt.track();
                double[] powers = aim.track(Positions.goalX, Positions.goalY, 1400, 850, 0, 0, 21.9);
                dt.drive(powers[0], powers[1], powers[2]);
            }
            dt.drive(0,0,0);
            timer.reset();
            bucket.resetIndexing();
            while(timer.milliseconds() < 700){
                bucket.index(true);
                bulk.tick(true, false);
                double[] powers = aim.track(Positions.goalX, Positions.goalY, 1400, 850, 0, 0, 21.9);
                dt.drive(powers[0], powers[1], powers[2]);
                dt.track();
            }
            shooter.cutPower();
            bucket.setIntaking();
            timer.reset();
            while(timer.milliseconds() < 500){
                bulk.tick(true, false);
                dt.track();
            }
            intake.setPower(1);
            timer.reset();
            while(timer.milliseconds() < 500){
                bulk.tick(true, false);
                dt.track();
            }
        }
        if(cPath == 5){
            if(fourring.getPaths().get(cPath).purePursuit(dt, this, .75, 1, 0)){
                cPath++;
            }
        }
        if(cPath == 6 && lPath == 5){
            wobble.lowerArm();
            timer.reset();
            while(timer.milliseconds() < 250){
                bulk.tick(true, false);
                dt.track();
            }
            intake.setPower(0);
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
            bucket.setHolding();
            shooter.spinUp();
            bucket.unJamBucket();
            timer.reset();
            RobotMovement.setPoint(new MovementPoint(1600, 850, 25), 1, 1);
            while(!RobotMovement.driveTowardPoint(dt, telemetry, true, 1)){
                bulk.tick(true, false);
                dt.track();
            }
            dt.drive(0,0,0);
            bucket.setShooting();
            timer.reset();
            while(timer.milliseconds() < 1000){
                bulk.tick(true, false);
                dt.track();
                double[] powers = aim.track(Positions.goalX, Positions.goalY, 1400, 850, 0, 0, 21.9);
                dt.drive(powers[0], powers[1], powers[2]);
            }
            timer.reset();
            bucket.resetIndexing();
            while(timer.milliseconds() < 1100){
                bucket.index(true);
                bulk.tick(true, false);
                double[] powers = aim.track(Positions.goalX, Positions.goalY, 1400, 850, 0, 0, 21.9);
                dt.drive(powers[0], powers[1], powers[2]);
                dt.track();
            }
            shooter.cutPower();
            bucket.setIntaking();
            RobotMovement.setPoint(new MovementPoint(1700, 850, 50), 1, 1);
            while(!RobotMovement.driveTowardPoint(dt, telemetry, true, 0) && opModeIsActive()){
                bulk.tick(true, false);
                dt.track();
            }
            dt.drive(0, 0, 0);
            dt.setOffsets(dt.getPosition().x, dt.getPosition().y, dt.getPosition().heading);
            while(opModeIsActive());
        }

        lPath = cPath;
        lPoint = fourring.getPaths().get(cPath).getCurrentPoint();
        telemetry.addData("x: ", dt.getPosition().x);
        telemetry.addData("y: ", dt.getPosition().y);
        telemetry.addData("h: ", dt.getPosition().heading * 360 / (2 * Math.PI));
    }

}
