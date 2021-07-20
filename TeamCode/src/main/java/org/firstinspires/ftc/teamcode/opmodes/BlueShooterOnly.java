package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.hardware.Wobble;
import org.firstinspires.ftc.teamcode.utils.AutoAimer;
import org.firstinspires.ftc.teamcode.utils.BulkReadHandler;
import org.firstinspires.ftc.teamcode.utils.MovementPoint;
import org.firstinspires.ftc.teamcode.utils.Positions;
import org.firstinspires.ftc.teamcode.utils.RingDetector;
import org.firstinspires.ftc.teamcode.utils.RobotMovement;

@Autonomous(name="BlueShooterOnly", group="blue")
public class BlueShooterOnly extends LinearOpMode
{

    private Drivetrain dt;
    private Wobble wobble;
    private Shooter shooter;
    private Intake intake;

    private AutoAimer aim;
    RingDetector detector;

    private BulkReadHandler bulk;

    private ElapsedTime timer;

    @Override
    public void runOpMode()
    {
        dt = new Drivetrain(this);
        wobble = new Wobble(true, this);
        shooter = new Shooter(this);
        intake = new Intake(this);

        aim = new AutoAimer(dt, shooter);

        detector = new RingDetector(this);

        bulk = new BulkReadHandler(this);

        dt.setOffsets(0, 0, 0);

        timer = new ElapsedTime();

        shooter.setHolding();
        shooter.setAngle(28);
        dt.getPosition().x = 270;
        dt.getPosition().y = 3200;
        dt.resetEncoder();

        waitForStart();

        int rings = detector.getDecision();
        detector.stopStreaming();
        intake.deployIntake();
        shooter.cutPower();

        MovementPoint point = new MovementPoint(1400, 3300, 50); //making the point


        RobotMovement.setPoint(point, 1, 1); //setting the point to move to
        while(!RobotMovement.driveTowardPoint(dt, telemetry, true, 0)){ //moving to the point
            bulk.tick(true, false);
            dt.track();
        }

        timer.reset(); //reset timer
        shooter.spinUp();
        shooter.setReleased();
        while(opModeIsActive() && timer.milliseconds() < 1500){
            bulk.tick(true, false);
            dt.track();
            double[] drive = aim.track(Positions.goalXb, Positions.goalYb, 1400, 3300, 0, 0, 0);
            dt.drive(drive[0], drive[1], drive[2]);
        }
        shooter.setIndexerPos(1);
        timer.reset(); //reset timer
        while(opModeIsActive() && timer.milliseconds() < 750){
            bulk.tick(true, false);
            dt.track();
            double[] drive = aim.track(Positions.goalXb, Positions.goalYb, 1400, 3300, 0, 0, 0);
            dt.drive(drive[0], drive[1], drive[2]);
        }
        shooter.setIndexerPos(2);
        timer.reset(); //reset timer
        while(opModeIsActive() && timer.milliseconds() < 750){
            bulk.tick(true, false);
            dt.track();
            double[] drive = aim.track(Positions.goalXb, Positions.goalYb, 1400, 3300, 0, 0, 0);
            dt.drive(drive[0], drive[1], drive[2]);
        }
        shooter.setIndexerPos(3);
        timer.reset(); //reset timer
        while(opModeIsActive() && timer.milliseconds() < 750){
            bulk.tick(true, false);
            dt.track();
            double[] drive = aim.track(Positions.goalXb, Positions.goalYb, 1400, 3300, 0, 0, 0);
            dt.drive(drive[0], drive[1], drive[2]);
        }
        timer.reset();
        shooter.cutPower();
        shooter.setIndexerPos(0);
        dt.drive(Math.toRadians(90), 1, 0);
        while(opModeIsActive() && timer.milliseconds() < 400){
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

}
