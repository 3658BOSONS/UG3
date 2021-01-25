package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Shooter;

@TeleOp(name="Turret Tuner", group="test")
public class TurretTuner extends LinearOpMode {


    Shooter shooter;

    @Override
    public void runOpMode()
    {
        shooter = new Shooter(this);

        waitForStart();

        shooter.resetTurretEncoder();

        shooter.setTurretAngle(2);

        sleep(2000);

        double pos = shooter.getTurretAngle();

        shooter.setTurretAngle(-2);

        sleep(2000);

        double pos2 = shooter.getTurretAngle();

        shooter.setTurretAngle(0);

        while(opModeIsActive()){
            telemetry.clear();
            telemetry.addData("SUM: ", pos - pos2);
            telemetry.addData("LEFT: ", pos);
            telemetry.addData("RIGHT: ", pos2);
            telemetry.update();
        }
    }

}
