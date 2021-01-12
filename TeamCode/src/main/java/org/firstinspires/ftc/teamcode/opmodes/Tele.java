package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Bucket;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.utils.State;

@TeleOp(name="TeleopRed", group = "Tele")
public class Tele extends LinearOpMode
{

    private Drivetrain dt;
    private Intake intake;
    private Shooter shooter;
    private Bucket bucket;

    private State state;

    private boolean lastBumper;
    private double targetAngle;

    private long lastTick;

    @Override
    public void runOpMode()
    {
        waitForStart();

        dt = new Drivetrain(this);
        intake = new Intake(this);
        shooter = new Shooter(this);
        bucket = new Bucket(this);

        state = State.INTAKING;
        lastBumper = false;
        targetAngle = 22;

        lastTick = 0;

        bucket.setIntaking();

        while(opModeIsActive())
        {

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
                    shooter.spinUp();
                    bucket.setShooting();
                    state = State.SHOOTING;
                }
                else if(state == State.SHOOTING)
                {
                    shooter.cutPower();
                    bucket.setIntaking();
                    state = State.INTAKING;
                }
            }

            lastBumper = bumper;

            if(state == State.INTAKING)
            {
                intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            }
            else if(state == State.SHOOTING)
            {
                bucket.index((gamepad1.right_trigger >= .3));
            }

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
        }
    }

}
