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
        cam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
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

    private static final float w = 640f;
    private static final float h = 480f;

    private static final Point TL = new Point(w*(1f/8f), h*(1f/8f));
    private static final Point BR = new Point(w*(7f/8f), h*(7f/8f));

    @Override
    public void onViewportTapped()
    {

    }

    @Override
    public Mat processFrame(Mat input)
    {

        Imgproc.rectangle(input, TL, BR, new Scalar(0, 255, 0), 2);

        process(TL, BR, input, color);

        tellemData();

        return input;
    }

    public int getDecision(){
        if(color.getB() < 30){
            return 0;
        }
        else if(color.getB() < 100){
            return 1;
        }
        else{
            return 4;
        }
    }

    public void process(Point p1, Point p2, Mat input, Color c)
    {
        int r = 0;
        int g = 0;
        int b = 0;
        int tot = 0;

        for(int x = (int)p1.x; x<(int)p2.x; x++)
        {
            for(int y = (int)p1.y; y<(int)p2.y; y++)
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
        op.telemetry.addLine("Decision: " + getDecision());
        op.telemetry.update();
    }
}
