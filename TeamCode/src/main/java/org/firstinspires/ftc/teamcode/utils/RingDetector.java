package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvTracker;

public class RingDetector
{

    private OpenCvCamera cam;
    private OpMode op;
    private Pipeline pipeline;
    private String pipe;

    public RingDetector(OpMode op)
    {
        this.op = op;
        this.pipe = pipe;
        this.pipeline = new Pipeline(op);

        int cameraMonitorViewId = op.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(op.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        cam.openCameraDevice();
        cam.setPipeline(pipeline);
        cam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
    }

    public void stopStreaming(){
        cam.stopStreaming();
    }

    public void pauseViewport(){
        cam.pauseViewport();
    }

    public void resumeViewport(){
        cam.resumeViewport();
    }

    public int getDecision(){
        return pipeline.getDecision();
    }

}

class Pipeline extends OpenCvPipeline
{

    OpMode op;

    public Pipeline(OpMode op){
        this.op = op;
    }

    private Color color = new Color();
    private Color lowerColor = new Color();

    private static final float w = 640f;
    private static final float h = 480f;

    private static final Point BR = new Point(w*(10f/16f), h*(11f/16f));
    private static final Point TL = new Point(w*(7f/16f), h*(13f/16f));

    private static final Point BR2 = new Point(w*(10f/16f), h*(26f/32f));
    private static final Point TL2 = new Point(w*(7f/16f), h*(29f/32f));

    @Override
    public void onViewportTapped()
    {

    }

    @Override
    public Mat processFrame(Mat input)
    {

        Imgproc.rectangle(input, TL, BR, new Scalar(0, 255, 0), 2);
        Imgproc.rectangle(input, TL2, BR2, new Scalar(0, 255, 0), 2);

        process(TL, BR, input, color);
        process(TL2, BR2, input, lowerColor);

        tellemData();

        return input;
    }

    public int getDecision(){
        int cutoff = 105;

        if(color.getB() < cutoff && lowerColor.getB() < cutoff){
            return 4;
        }
        else if(lowerColor.getB() < cutoff){
            return 1;
        }
        else{
            return 0;
        }
    }

    public void process(Point p1, Point p2, Mat input, Color c)
    {
        int r = 0;
        int g = 0;
        int b = 0;
        int tot = 1;

        for(int x = (int)p1.x; x<(int)p2.x; x+=2)
        {
            for(int y = (int)p2.y; y<(int)p1.y; y+=2)
            {
                r += input.get(y, x)[0];
                g += input.get(y, x)[1];
                b += input.get(y, x)[2];
                tot++;
            }
        }
        r/=tot;
        g/=tot;
        b/=tot;


        c.setR(r);
        c.setG(g);
        c.setB(b);
    }

    private void tellemData(){
        op.telemetry.clear();
        op.telemetry.addLine("Color:  R" + color.getR() + " G " + color.getG() + " B " + color.getB());
        op.telemetry.addLine("Lower:  R" + lowerColor.getR() + " G " + lowerColor.getG() + " B " + lowerColor.getB());
        op.telemetry.addLine("Decision: " + getDecision());
        op.telemetry.update();
    }
}
