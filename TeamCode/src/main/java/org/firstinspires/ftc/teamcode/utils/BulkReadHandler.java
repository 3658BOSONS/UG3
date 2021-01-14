package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BulkReadHandler
{

    LynxModule chub;
    LynxModule ehub;
    private ElapsedTime timer;

    public BulkReadHandler(LinearOpMode opMode)
    {
        chub = opMode.hardwareMap.get(LynxModule.class, "Control Hub");
        ehub = opMode.hardwareMap.get(LynxModule.class, "Expansion Hub 3");

        chub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        timer = new ElapsedTime();
        timer.reset();
    }

    public double tick()
    {
        chub.getBulkData();
        ehub.getBulkData();
        double hz = 1000 / timer.milliseconds();
        timer.reset();
        return hz;
    }

}
