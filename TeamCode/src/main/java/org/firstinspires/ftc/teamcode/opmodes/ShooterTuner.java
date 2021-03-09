package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.utils.BulkReadHandler;

@TeleOp(name="Shooter Tuner", group="test")
@Config
public class ShooterTuner  extends LinearOpMode {

    private Shooter shooter;
    private BulkReadHandler bulk;
    private FtcDashboard dashboard;

    public static double P = 150;
    public static double I = 4;
    public static double D = 1;

    private boolean lastR;
    private boolean shooting;

    @Override
    public void runOpMode(){
        shooter = new Shooter(this);
        lastR = false;
        shooting = false;
        bulk = new BulkReadHandler(this);
        dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()){
            boolean r = gamepad1.right_bumper;
            if(r && !lastR){
                if(shooting){
                    shooter.cutPower();
                    shooting = false;
                }else{
                    shooter.setPid(P, I, D);
                    shooter.spinUp();
                    shooting = true;
                }
            }
            lastR = r;

            bulk.tick(false, true);
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("VELO: ", shooter.getShooterVelo());
            packet.put("TARGET: ", 4000);
            dashboard.sendTelemetryPacket(packet);
        }

    }

}
