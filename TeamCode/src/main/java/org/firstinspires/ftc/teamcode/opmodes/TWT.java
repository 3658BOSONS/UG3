package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Gyro;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.OdoWheel;
import org.firstinspires.ftc.teamcode.utils.BulkReadHandler;

@TeleOp(name="track width", group="test")
public class TWT extends LinearOpMode {


    private Drivetrain dt;
    private OdoWheel left;
    private OdoWheel right;
    private Gyro gyro;
    private BulkReadHandler bulk;

    @Override
    public void runOpMode()
    {

        waitForStart();

        dt = new Drivetrain(this);
        left = new OdoWheel(false, 58, new Motor(this, "fl"));
        right = new OdoWheel(false, 58, new Motor(this, "br"));
        gyro = new Gyro(this);
        bulk = new BulkReadHandler(this);

        while(opModeIsActive())
        {
            bulk.tick(true, false);
            dt.drive(0, 0, .75);
            double angle = Math.toRadians(gyro.getAngle());
            double trackWidth = (right.getMM() - left.getMM()) / angle;

            telemetry.clear();
            telemetry.addData("l " , left.getMM());
            telemetry.addData("r " , right.getMM());
            telemetry.addData("gyro", gyro.getAngle());
            telemetry.addData("TW: ", trackWidth);
            telemetry.update();
        }

    }

}
