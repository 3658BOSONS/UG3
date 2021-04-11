package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.hardware.Wobble;
import org.firstinspires.ftc.teamcode.utils.AutoAimer;
import org.firstinspires.ftc.teamcode.utils.BosonMath;
import org.firstinspires.ftc.teamcode.utils.BulkReadHandler;
import org.firstinspires.ftc.teamcode.utils.MovementPoint;
import org.firstinspires.ftc.teamcode.utils.PIDF;
import org.firstinspires.ftc.teamcode.utils.Positions;
import org.firstinspires.ftc.teamcode.utils.RobotMovement;
import org.firstinspires.ftc.teamcode.utils.State;

@TeleOp(name="TeleopRed", group = "Tele")
public class Tele extends LinearOpMode
{

    private Drivetrain dt;
    private Intake intake;
    private Shooter shooter;
    private Wobble wobble;

    private BulkReadHandler bulk;

    private State state;

    private AutoAimer aim;

    private boolean lastBumper;
    private boolean lastX;
    private boolean isGripping;
    private boolean lastDpad;

    private double spotX = 1700;
    private double spotY = 850;

    private double psSpotX = 1600;
    private double psSpotY = 1300;

    private int currentPowershot;
    private boolean lastL;

    private double distanceOffset;
    private double sidewaysOffset;

    private double lastRight;
    private int lastIndex;

    @Override
    public void runOpMode()
    {
        waitForStart();

        dt = new Drivetrain(this);
        intake = new Intake(this);
        shooter = new Shooter(this);
        wobble = new Wobble(false, this);
        wobble.resetArm();

        dt.getPosition().x = dt.xOffset;
        dt.getPosition().y = dt.yOffset;

        aim = new AutoAimer(dt, shooter);
        shooter.cutPower();

        state = State.INTAKING;
        lastBumper = false;

        distanceOffset = 0;
        sidewaysOffset = 0;

        lastRight = 0;
        lastIndex = 0;

        lastX = false;
        isGripping = false;

        currentPowershot = -1;
        lastL = false;

        bulk = new BulkReadHandler(this);

        while(opModeIsActive())
        {
            telemetry.clear();
            double looptime = bulk.tick(true, false);
            dt.track();

            boolean bumper;
            bumper = gamepad1.right_bumper;

            if(gamepad1.left_bumper){
                state = State.POWERSHOT;
                shooter.spinPowershots();
                shooter.setReleased();
            }

            if(bumper && !lastBumper)
            {
                if(state == State.INTAKING)
                {
                    shooter.waitThenRaiseChute();
                    shooter.setHolding();
                    state = State.HOLDING;
                }
                else if(state == State.HOLDING)
                {
                    shooter.spinUp();
                    shooter.setReleased();
                    state = State.SHOOTING;
                }
                else if(state == State.SHOOTING)
                {
                    shooter.cutPower();
                    shooter.setAngle(-10);
                    state = State.INTAKING;
                    wobble.resetArm();
                }
                else if(state == State.POWERSHOT){
                    shooter.cutPower();
                    shooter.setAngle(-10);
                    state = State.INTAKING;
                    wobble.resetArm();
                }
            }

            lastBumper = bumper;
            double[] dtValues = new double[]{0,0,0,0};

            if(state == State.INTAKING)
            {
                intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            }
            else if(state == State.HOLDING)
            {

            }
            else if(state == State.SHOOTING)
            {

                double distToPoint = Math.hypot(spotX - dt.getPosition().x, spotY - dt.getPosition().y);

                if(distToPoint > 100){
                    RobotMovement.setPoint(new MovementPoint(spotX, spotY, 50), 1, 1);
                    RobotMovement.driveTowardPoint(dt, telemetry, true, 1);
                }

                dtValues = aim.track(Positions.goalX, Positions.goalY, spotX, spotY, distanceOffset, sidewaysOffset, 0);

                double right = gamepad1.right_trigger;
                if(right >= .3){
                    shooter.setIndexerPos(3);
                }else{
                    shooter.setIndexerPos(0);
                }
                lastRight = right;
            }
            else if(state == State.POWERSHOT)
            {
                double goalPosX;
                double goalPosY;
                if(currentPowershot == 0){
                    goalPosX = Positions.psRightX;
                    goalPosY = Positions.psRightY;
                }
                else if(currentPowershot == 1){
                    goalPosX = Positions.psMidX;
                    goalPosY = Positions.psMidY;
                }
                else{
                    goalPosX = Positions.psLeftX;
                    goalPosY = Positions.psLeftY;
                }

                dtValues = aim.track(goalPosX, goalPosY, psSpotX, psSpotY, distanceOffset, sidewaysOffset, -.5);

                double right = gamepad1.right_trigger;
                if(right >= .3 && lastRight < .2){
                    if(lastIndex == 0){
                        shooter.setIndexerPos(1);
                        lastIndex = 1;
                    }else if (lastIndex == 1){
                        shooter.setIndexerPos(2);
                        lastIndex = 2;
                    }
                    else if (lastIndex == 2){
                        shooter.extendThenRetract();
                        lastIndex = 0;
                    }
                }
                lastRight = right;
                boolean l = gamepad1.left_bumper;
                if(l && !lastL){
                    currentPowershot++;
                }
                if(currentPowershot > 2){
                    currentPowershot = 0;
                }
                lastL = l;
                telemetry.addData("PS:  ", currentPowershot);
            }

            //dtValues = new double[]{0,0,0,0}; //OVERRIDE AUTOAIM**********
            double x = gamepad1.left_stick_x * 1.5;
            double y = -gamepad1.left_stick_y;
            if(gamepad1.right_stick_x != 0)
                dtValues[2] = -gamepad1.right_stick_x;
            if(x != 0 || y != 0){
                dtValues[1] = Math.hypot(x, y);
                dtValues[0] = Math.atan2(y, x);
            }

            dt.drive(dtValues[0], dtValues[1], dtValues[2]);

            if(gamepad1.left_stick_button && gamepad1.right_stick_button){
                dt.resetEncoder();
                dt.setOffsets(0,0,0);
                dt.getPosition().x = 455/2;
                dt.getPosition().y = 455/2;
                dt.getPosition().heading = 0;
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

            if(gamepad1.dpad_up && !lastDpad){
                dt.getPosition().x = dt.getPosition().x - 20;
                lastDpad = true;
            }
            else if(gamepad1.dpad_down && !lastDpad){
                dt.getPosition().x = dt.getPosition().x +  20;
                lastDpad = true;
            }
            else if(gamepad1.dpad_right && !lastDpad){
                if(gamepad1.left_trigger > .3){
                    dt.headingOffset -= Math.toRadians(1);
                    lastDpad = true;
                }else{
                    dt.getPosition().y = dt.getPosition().y + 15;
                    lastDpad = true;
                }
            }
            else if(gamepad1.dpad_left && !lastDpad){
                if(gamepad1.left_trigger > .3){
                    dt.headingOffset += Math.toRadians(1);
                    lastDpad = true;
                }else{
                    dt.getPosition().y = dt.getPosition().y - 15;
                    lastDpad = true;
                }
            }

            if(!gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_down && !gamepad1.dpad_up){
                lastDpad = false;
            }

            telemetry.addData("Hz: ", looptime);
            //bulk.tick(false, true);
            //telemetry.addData("shooter: ", shooter.getShooterVelo());
            telemetry.addData("H ERROR: ", Math.toDegrees(dtValues[3]));
            telemetry.addData("x: ", dt.getPosition().x);
            telemetry.addData("y: ", dt.getPosition().y);
            telemetry.addData("h: ", dt.getPosition().heading * 360 / (2 * Math.PI));
            telemetry.update();
        }
    }

}
