package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.utils.BulkReadHandler;
import org.firstinspires.ftc.teamcode.utils.PIDF;

@TeleOp(name="Heading Tuner", group="test")
@Config
public class HeadingPIDTuner extends LinearOpMode {

    private Drivetrain dt;
    private Shooter shooter;
    private BulkReadHandler bulk;
    private FtcDashboard dashboard;

    public static double P = 3;
    public static double I = 0;
    public static double D = 1;

    private PIDF heading;

    private boolean lastR;
    private boolean manual;

    @Override
    public void runOpMode(){
        dt = new Drivetrain(this);
        dt.resetEncoder();
        lastR = false;
        manual = true;
        bulk = new BulkReadHandler(this);
        dashboard = FtcDashboard.getInstance();
        heading = new PIDF(P, I, D, 0);
        shooter = new Shooter(this);

        waitForStart();

        while (opModeIsActive()){
            boolean r = gamepad1.right_bumper;
            if(r && !lastR){
                if(manual){
                    manual = false;
                    heading = new PIDF(P, I, D, 0);
                    heading.setTarget(0);
                }else{
                    manual = true;
                }
            }
            lastR = r;

            if(manual){
                double x = gamepad1.left_stick_x * 1.5;
                double y = -gamepad1.left_stick_y;
                dt.drive(Math.atan2(y, x), Math.hypot(x, y), -gamepad1.right_stick_x);
            }else{
                dt.drive(0, 0, heading.tick(dt.getPosition().heading));
            }

            bulk.tick(true, false);
            dt.track();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("HEADING: ", dt.getPosition().heading);
            packet.put("TARGET: ", 0);
            dashboard.sendTelemetryPacket(packet);
        }

    }

}
