package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Bucket;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.hardware.Wobble;
import org.firstinspires.ftc.teamcode.utils.BulkReadHandler;
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

    private boolean lastBumper;
    private double targetAngle;
    private boolean lastB;
    private double turretTarget;
    private boolean lastX;
    private boolean isGripping;

    private long lastTick;

    @Override
    public void runOpMode()
    {
        waitForStart();

        dt = new Drivetrain(this);
        intake = new Intake(this);
        shooter = new Shooter(this);
        bucket = new Bucket(this);
        wobble = new Wobble(false, this);

        state = State.INTAKING;
        lastBumper = false;
        targetAngle = 30;
        turretTarget = 0;

        lastX = false;
        isGripping = false;

        lastTick = 0;

        bucket.setIntaking();

        bulk = new BulkReadHandler(this);

        while(opModeIsActive())
        {
            double tickrate = bulk.tick(true, false);
            dt.track();

            double x = gamepad1.left_stick_x * 1.5;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            double p = Math.sqrt((x * x) + (y * y));
            double theta = Math.atan2(y, x);

            dt.drive(theta, p, turn);

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
                bucket.index((gamepad1.right_trigger >= .3));
                boolean b = gamepad1.b;
                if(b && !lastB)
                {
                    bucket.unJamMag();
                }
                lastB = b;
            }
            if(state == State.SHOOTING || state == State.HOLDING);
            {
                turretTarget += (gamepad1.dpad_right ? 1 : 0) * .01;
                turretTarget -= (gamepad1.dpad_left ? 1 : 0) * .01;
                shooter.setTurretTarget(turretTarget);
            }

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

            double input = 0;
            if(gamepad1.dpad_up)
                input = 1;
            else if(gamepad1.dpad_down)
                input = -1;

            input *= 23;
            input *= .003;

            targetAngle += input;

            if(targetAngle > 45)
                targetAngle = 45;
            if(targetAngle < 22)
                targetAngle = 22;

            shooter.setAngle(targetAngle);
            double sPower = shooter.tickTurret();

            telemetry.clear();
            telemetry.addData("Hz: ", tickrate);
            /*telemetry.addData("turret: ", shooter.getTurretPosition());
            telemetry.addData("shooter: ", shooter.getShooterVelo());
            telemetry.addData("turret power: ", sPower);*/
            telemetry.addData("x: ", dt.getPosition().x);
            telemetry.addData("y: ", dt.getPosition().y);
            telemetry.addData("h: ", dt.getPosition().heading * 360 / (2 * Math.PI));
            telemetry.update();
        }
    }

}
