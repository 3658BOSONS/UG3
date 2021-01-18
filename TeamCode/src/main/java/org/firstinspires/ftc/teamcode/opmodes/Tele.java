package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Bucket;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.hardware.Wobble;
import org.firstinspires.ftc.teamcode.utils.BosonMath;
import org.firstinspires.ftc.teamcode.utils.BulkReadHandler;
import org.firstinspires.ftc.teamcode.utils.PIDF;
import org.firstinspires.ftc.teamcode.utils.State;

@TeleOp(name="TeleopRed", group = "Tele")
public class Tele extends LinearOpMode
{

    private Drivetrain dt;
    private Intake intake;
    private Shooter shooter;
    private Bucket bucket;
    private Wobble wobble;

    private BulkReadHandler bulk;

    private State state;

    private PIDF heading;

    private boolean lastBumper;
    private double targetAngle;
    private boolean lastB;
    private double turretTarget;
    private boolean lastX;
    private boolean isGripping;
    private boolean lastDpad;

    private double goalX = 3660;
    private double goalY = 700;

    private long lastTick;

    @Override
    public void runOpMode()
    {
        waitForStart();

        dt = new Drivetrain(this);
        dt.getPosition().x = 2130;
        dt.getPosition().y = 914;
        intake = new Intake(this);
        shooter = new Shooter(this);
        bucket = new Bucket(this);
        wobble = new Wobble(false, this);

        heading = new PIDF(2.5, 0, .25, 0);

        state = State.INTAKING;
        lastBumper = false;
        targetAngle = 33;
        turretTarget = 0;

        lastX = false;
        isGripping = false;

        lastTick = 0;

        bucket.setIntaking();

        bulk = new BulkReadHandler(this);

        while(opModeIsActive())
        {
            telemetry.clear();
            double looptime = bulk.tick(true, false);
            dt.track();

            boolean bumper = false;
            bumper = gamepad1.right_bumper;
            if(bumper && !lastBumper)
            {
                if(state == State.INTAKING)
                {
                    bucket.setHolding();
                    state = State.HOLDING;
                    wobble.lowerArm();
                }
                else if(state == State.HOLDING)
                {
                    shooter.spinUp();
                    bucket.setShooting();
                    state = State.SHOOTING;
                }
                else if(state == State.SHOOTING)
                {
                    shooter.cutPower();
                    bucket.setIntaking();
                    state = State.INTAKING;
                    wobble.raiseArm();
                }
            }

            lastBumper = bumper;
            double turn = 0;

            if(state == State.INTAKING)
            {
                intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                shooter.setTurretTarget(0);
                turretTarget = 0;
            }
            else if(state == State.HOLDING)
            {
                boolean b = gamepad1.b;
                if(b && !lastB)
                {
                    bucket.unJamBucket();
                }
                lastB = b;
            }
            else if(state == State.SHOOTING)
            {
                double xToGoal = goalX - dt.getPosition().x;
                double yToGoal = goalY - dt.getPosition().y;
                double distanceToGoal = Math.hypot(xToGoal, yToGoal);
                double angleToGoal = Math.atan2(yToGoal, xToGoal);
                double dtAngle = BosonMath.clipAngle(dt.getPosition().heading);
                heading.setTarget(angleToGoal);
                telemetry.addData("atg ", angleToGoal);
                telemetry.addData("dta ", dtAngle);
                turn = heading.tick(dtAngle);

                targetAngle = (1.0/380.0)*distanceToGoal + 28.05;

                bucket.index((gamepad1.right_trigger >= .3));
                boolean b = gamepad1.b;
                if(b && !lastB)
                {
                    bucket.unJamMag();
                }
                lastB = b;
            }

            double x = gamepad1.left_stick_x * 1.5;
            double y = -gamepad1.left_stick_y;
            if(gamepad1.right_stick_x != 0)
                turn = -gamepad1.right_stick_x;

            double p = Math.sqrt((x * x) + (y * y));
            double theta = Math.atan2(y, x);

            dt.drive(theta, p, turn);

            if(gamepad1.a){
                wobble.lowerArm();
            }
            if(gamepad1.y){
                wobble.raiseArm();
            }
            boolean buttonx = gamepad1.x;
            if(buttonx && !lastX){
                if(!isGripping){
                    wobble.grip();
                    isGripping = true;
                }
                else{
                    wobble.release();
                    isGripping = false;
                }
            }
            lastX = buttonx;

            if(gamepad1.dpad_up && !lastDpad){
                goalX += 25;
                lastDpad = true;
            }
            else if(gamepad1.dpad_down && !lastDpad){
                goalX -= 25;
                lastDpad = true;
            }
            else if(gamepad1.dpad_right && !lastDpad){
                goalY -= 25;
                lastDpad = true;
            }
            else if(gamepad1.dpad_left && !lastDpad){
                goalY += 25;
                lastDpad = true;
            }else{
                lastDpad = false;
            }

            shooter.setAngle(targetAngle);
            double sPower = shooter.tickTurret();

            telemetry.addData("Hz: ", looptime);
            telemetry.addData("angle ", targetAngle);
           // bulk.tick(false, true);
            //telemetry.addData("shooter: ", shooter.getShooterVelo());
            telemetry.addData("turret power: ", sPower);
            //telemetry.addData("x: ", dt.getPosition().x);
            //telemetry.addData("y: ", dt.getPosition().y);
            //telemetry.addData("h: ", dt.getPosition().heading * 360 / (2 * Math.PI));
            telemetry.update();
        }
    }

}
