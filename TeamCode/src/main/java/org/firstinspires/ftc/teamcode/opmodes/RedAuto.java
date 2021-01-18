package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Bucket;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.hardware.Wobble;
import org.firstinspires.ftc.teamcode.utils.BulkReadHandler;
import org.firstinspires.ftc.teamcode.utils.PIDF;

@Autonomous(name="RED", group="auto")
public class RedAuto extends LinearOpMode
{

    private Drivetrain dt;
    private Wobble wobble;
    private Shooter shooter;
    private Bucket bucket;
    private Intake intake;

    private BulkReadHandler bulk;

    private PIDF autoAimHeading;

    private ElapsedTime timer;

    @Override
    public void runOpMode()
    {
        dt = new Drivetrain(this);
        wobble = new Wobble(true, this);
        shooter = new Shooter(this);
        bucket = new Bucket(this);
        intake = new Intake(this);

        bulk = new BulkReadHandler(this);

        autoAimHeading = new PIDF(2.5, 0, .25, 0);

        timer = new ElapsedTime();

        bucket.setHolding();
        bucket.deactivateOrange();
        dt.getPosition().x = 450;
        dt.getPosition().y = 1000;
        shooter.setAngle(45);

        waitForStart();
        bucket.setShooting();
        shooter.spinUp();
        sleep(1500);
        bucket.index(true);
        timer.reset();
        while(timer.milliseconds() < 600){
            bucket.index(false);
        }
        autoAimHeading.setTarget(Math.toRadians(3));
        timer.reset();
        while(timer.milliseconds() < 1000){
            bulk.tick(true, false);
            dt.track();
            dt.drive(Math.toRadians(90), .15, autoAimHeading.tick(dt.getPosition().heading));
        }
        dt.drive(0,0,0);
        bucket.index(true);
        timer.reset();
        while(timer.milliseconds() < 600){
            bucket.index(false);
        }
        autoAimHeading.setTarget(Math.toRadians(6));
        timer.reset();
        while(timer.milliseconds() < 1000){
            bulk.tick(true, false);
            dt.track();
            dt.drive(Math.toRadians(90), .1, autoAimHeading.tick(dt.getPosition().heading));
        }
        dt.drive(0,0,0);
        bucket.index(true);
        timer.reset();
        while(timer.milliseconds() < 600){
            bucket.index(false);
        }
    }

}
