package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Bucket;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.hardware.Wobble;
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
    PurePursuit powerDrive;

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

        timer = new ElapsedTime();

        powerShots = false;
        cPath = 0;

        bucket.setHolding();
        bucket.deactivateOrange();
        dt.getPosition().x = 230;
        dt.getPosition().y = 990;
        shooter.setAngle(22);
        shooter.setTurretAngle(0);

        ArrayList<MovementPoint> psDrive = new ArrayList<>();
        psDrive.add(new MovementPoint(450, 1000, 0));
        psDrive.add(new MovementPoint(1200, 1800, 0));
        psDrive.add(new MovementPoint(1600, 1300, 100));

        powerDrive = new PurePursuit(psDrive);

        waitForStart();

        //READ DISCS
        int rings = detector.getDecision();
        detector.stopStreaming();

        while(opModeIsActive())
        {
            telemetry.clear();
            bulk.tick(true, false);
            dt.track();

            if(!powerShots){
               tickPowerShots();
            }
            else{

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
            while(opModeIsActive() && timer.milliseconds() < 1000){
                bulk.tick(true, false);
                dt.track();
                double[] drive = aim.track(Positions.psRightX, Positions.psRightY, 1600, 1300, 0, 0, 21.0);
                dt.drive(drive[0], drive[1], drive[2]);
            }
            timer.reset();
            while(opModeIsActive() && timer.milliseconds() < 300){
                bulk.tick(true, false);
                dt.track();
                bucket.index(true);
                double[] drive = aim.track(Positions.psRightX, Positions.psRightY, 1600, 1300, 0, 0, 21.0);
                dt.drive(drive[0], drive[1], drive[2]);
            }

            while(opModeIsActive() && timer.milliseconds() < 1000){
                bulk.tick(true, false);
                dt.track();
                double[] drive = aim.track(Positions.psMidX, Positions.psMidY, 1600, 1300, 0, 0, 21.0);
                dt.drive(drive[0], drive[1], drive[2]);
            }
            timer.reset();
            while(opModeIsActive() && timer.milliseconds() < 300){
                bulk.tick(true, false);
                dt.track();
                bucket.index(true);
                double[] drive = aim.track(Positions.psMidX, Positions.psMidY, 1600, 1300, 0, 0, 21.0);
                dt.drive(drive[0], drive[1], drive[2]);
            }

            while(opModeIsActive() && timer.milliseconds() < 1000){
                bulk.tick(true, false);
                dt.track();
                double[] drive = aim.track(Positions.psLeftX, Positions.psLeftY, 1600, 1300, 0, 0, 21.0);
                dt.drive(drive[0], drive[1], drive[2]);
            }
            timer.reset();
            while(opModeIsActive() && timer.milliseconds() < 300){
                bulk.tick(true, false);
                dt.track();
                bucket.index(true);
                double[] drive = aim.track(Positions.psLeftX, Positions.psLeftY, 1600, 1300, 0, 0, 21.0);
                dt.drive(drive[0], drive[1], drive[2]);
            }
            powerShots = true;
            bucket.setIntaking();
            shooter.setTurretAngle(0);
            shooter.cutPower();
        }
    }

}
