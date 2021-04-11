package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private Intake intake;

    private AutoAimer aim;
    RingDetector detector;

    private BulkReadHandler bulk;

    private ElapsedTime timer;
    private final double angle = 24.6;

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
        shooter = new Shooter(this);        intake = new Intake(this);

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

        shooter.setHolding();
        shooter.setAngle(28);
        dt.getPosition().x = 230;
        dt.getPosition().y = 990;
        dt.resetEncoder();

        ArrayList<MovementPoint> psDrive = new ArrayList<>();
        psDrive.add(new MovementPoint(450, 1000, 0));
        psDrive.add(new MovementPoint(550, 1600, 0));
        psDrive.add(new MovementPoint(1200, 1300, 100));

        powerDrive = new PurePursuit(psDrive);

        waitForStart();

        int rings = detector.getDecision();
        detector.stopStreaming();
        intake.deployIntake();

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
            shooter.spinPowershots();
            shooter.setReleased();
            while(opModeIsActive() && timer.milliseconds() < 1250){
                bulk.tick(true, false);
                dt.track();
                double[] drive = aim.track(Positions.psRightX, Positions.psRightY, 1600, 1300, 0, 0, -1.5);
                dt.drive(drive[0], drive[1], drive[2]);
            }
            shooter.setIndexerPos(1);
            timer.reset();
            while(opModeIsActive() && timer.milliseconds() < 300){
                bulk.tick(true, false);
                dt.track();
                double[] drive = aim.track(Positions.psRightX, Positions.psRightY, 1600, 1300, 0, 0, -1.5);
                dt.drive(drive[0], drive[1], drive[2]);
            }
            while(opModeIsActive() && timer.milliseconds() < 800){
                bulk.tick(true, false);
                dt.track();
                double[] drive = aim.track(Positions.psMidX, Positions.psMidY, 1600, 1300, 0, 0, -1);
                dt.drive(drive[0], drive[1], drive[2]);
            }
            shooter.setIndexerPos(2);
            timer.reset();
            while(opModeIsActive() && timer.milliseconds() < 400){
                bulk.tick(true, false);
                dt.track();
                double[] drive = aim.track(Positions.psMidX, Positions.psMidY, 1600, 1300, 0, 0, -1);
                dt.drive(drive[0], drive[1], drive[2]);
            }
            while(opModeIsActive() && timer.milliseconds() < 800){
                bulk.tick(true, false);
                dt.track();
                double[] drive = aim.track(Positions.psLeftX, Positions.psLeftY, 1600, 1300, 0, 0, -.5);
                dt.drive(drive[0], drive[1], drive[2]);
            }
            shooter.setIndexerPos(3);
            timer.reset();
            while(opModeIsActive() && timer.milliseconds() < 750){
                bulk.tick(true, false);
                dt.track();
                double[] drive = aim.track(Positions.psLeftX, Positions.psLeftY, 1600, 1300, 0, 0, -.5);
                dt.drive(drive[0], drive[1], drive[2]);
            }
            shooter.setIndexerPos(0);
            timer.reset();
            while(timer.milliseconds() < 500){
                bulk.tick(true, false);
                dt.track();
            }
            shooter.setAngle(-10);
            shooter.cutPower();
            powerShots = true;
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
            while(timer.milliseconds() < 800){
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
            timer.reset();
            while(timer.milliseconds() < 1500 && opModeIsActive()){
                bulk.tick(true, false);
                dt.trackWithoutOffsets();
                dt.setOffsets(dt.getPosition().x, dt.getPosition().y, dt.getPosition().heading);
                dt.readPose(telemetry);
            }
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
            while(timer.milliseconds() < 800){
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
            wobble.resetArm();
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
            shooter.setHolding();
            shooter.waitThenRaiseChute();
            shooter.spinUp();
            timer.reset();
            while(timer.milliseconds() < 1000){
                bulk.tick(true, false);
                dt.track();
                double[] powers = aim.track(Positions.goalX, Positions.goalY, 1700, 850, 0, 0, 0);
                dt.drive(powers[0], powers[1], powers[2]);
            }
            shooter.setReleased();
            timer.reset();
            while(timer.milliseconds() < 750){
                bulk.tick(true, false);
                dt.track();
                double[] powers = aim.track(Positions.goalX, Positions.goalY, 1700, 850, 0, 0, 0);
                dt.drive(powers[0], powers[1], powers[2]);
            }
            shooter.setIndexerPos(3);
            timer.reset();
            while(timer.milliseconds() < 750){
                bulk.tick(true, false);
                dt.track();
            }
            shooter.cutPower();
            shooter.setIndexerPos(0);
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
            while(timer.milliseconds() < 1500 && opModeIsActive()){
                bulk.tick(true, false);
                dt.trackWithoutOffsets();
                dt.setOffsets(dt.getPosition().x, dt.getPosition().y, dt.getPosition().heading);
                dt.readPose(telemetry);
            }
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
            while(timer.milliseconds() < 800){
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
            wobble.resetArm();
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

        }
        if(cPath == 4){
            if(fourring.getPaths().get(cPath).purePursuit(dt, this, .4, 1, 0)){
                cPath++;
            }
        }
        if(cPath == 5 && lPath == 4){
            timer.reset();
            while(timer.milliseconds() < 500){
                bulk.tick(true, false);
                dt.track();
            }
            shooter.setHolding();
            shooter.waitThenRaiseChute();
            timer.reset();
            while(timer.milliseconds() < 250){
                bulk.tick(true, false);
                dt.track();
            }
            shooter.spinUp();
            intake.setPower(-.2);
            timer.reset();
            while(timer.milliseconds() < 750){
                bulk.tick(true, false);
                dt.track();
                double[] powers = aim.track(Positions.goalX, Positions.goalY, 1700, 850, 0, 0, 0);
                dt.drive(powers[0], powers[1], powers[2]);
            }
            shooter.setReleased();
            timer.reset();
            while(timer.milliseconds() < 750){
                bulk.tick(true, false);
                dt.track();
                double[] powers = aim.track(Positions.goalX, Positions.goalY, 1700, 850, 0, 0, 0);
                dt.drive(powers[0], powers[1], powers[2]);
            }
            timer.reset();
            shooter.setIndexerPos(3);
            while(timer.milliseconds() < 750){
                bulk.tick(true, false);
                double[] powers = aim.track(Positions.goalX, Positions.goalY, 1700, 850, 0, 0, 0);
                dt.drive(powers[0], powers[1], powers[2]);
                dt.track();
            }
            shooter.cutPower();
            shooter.setIndexerPos(0);
            timer.reset();
            while(timer.milliseconds() < 300){
                bulk.tick(true, false);
                dt.track();
            }
            shooter.setAngle(-10);
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
            while(timer.milliseconds() < 700){
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
            wobble.resetArm();
            while(timer.milliseconds() < 250){
                bulk.tick(true, false);
                dt.track();
                dt.drive(-Math.PI/2, 1, 0);
            }
            shooter.setHolding();
            shooter.waitThenRaiseChute();
            shooter.spinUp();
            timer.reset();
            RobotMovement.setPoint(new MovementPoint(1675, 850, 25), .75, 1);
            while(!RobotMovement.driveTowardPoint(dt, telemetry, true, 1)){
                bulk.tick(true, false);
                dt.track();
            }
            dt.drive(0,0,0);
            shooter.setReleased();
            timer.reset();
            while(timer.milliseconds() < 1000){
                bulk.tick(true, false);
                dt.track();
                double[] powers = aim.track(Positions.goalX, Positions.goalY, 1700, 850, 0, 0, 0);
                dt.drive(powers[0], powers[1], powers[2]);
            }
            timer.reset();
            shooter.setIndexerPos(3);
            while(timer.milliseconds() < 850){
                bulk.tick(true, false);
                double[] powers = aim.track(Positions.goalX, Positions.goalY, 1700, 850, 0, 0, 0);
                dt.drive(powers[0], powers[1], powers[2]);
                dt.track();
            }
            shooter.cutPower();
            shooter.setIndexerPos(0);
            RobotMovement.setPoint(new MovementPoint(1750, 850, 50), 1, 1);
            while(!RobotMovement.driveTowardPoint(dt, telemetry, true, 0) && opModeIsActive()){
                bulk.tick(true, false);
                dt.track();
            }
            wobble.lowerArm();
            dt.drive(0, 0, 0);
            while(timer.milliseconds() < 1500 && opModeIsActive()){
                bulk.tick(true, false);
                dt.trackWithoutOffsets();
                dt.setOffsets(dt.getPosition().x, dt.getPosition().y, dt.getPosition().heading);
                dt.readPose(telemetry);
            }
            while(opModeIsActive());
        }

        lPath = cPath;
        lPoint = fourring.getPaths().get(cPath).getCurrentPoint();
        telemetry.addData("x: ", dt.getPosition().x);
        telemetry.addData("y: ", dt.getPosition().y);
        telemetry.addData("h: ", dt.getPosition().heading * 360 / (2 * Math.PI));
    }

}
