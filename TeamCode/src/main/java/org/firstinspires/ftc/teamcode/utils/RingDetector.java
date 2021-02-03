package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvTracker;

import java.util.ArrayList;
import java.util.List;

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
        this.pipeline = new Pipeline(op.telemetry);

        int cameraMonitorViewId = op.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(op.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        cam.openCameraDevice();
        cam.setPipeline(pipeline);
        cam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
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

    private Telemetry telemetry;
    private final Scalar lowerOrange = new Scalar(0.0, 141.0, 0.0);
    private final Scalar upperOrange = new Scalar(255.0, 235.0, 90.0);
    private final int CAMERA_WIDTH = 320;
    private final int HORIZON = (int) (160.0 / 320.0 * (double) CAMERA_WIDTH);
    private final int MIN_WIDTH = (int) (50.0 / 320.0 * (double) CAMERA_WIDTH);
    private final double BOUND_RATIO = 0.73;

    private Mat mat;
    private Mat ret;
    private int height;

    public Pipeline(Telemetry telemetry)
    {
        this.telemetry = telemetry;
        this.height = 0;
        this.mat = new Mat();
        this.ret = new Mat();
    }

    @Override
    public void onViewportTapped()
    {

    }

    @Override
    public Mat processFrame(Mat input)
    {
        ret.release();
        ret = new Mat();

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb); //converting color spaces or something weird help me jaran

        Mat mask = new Mat(mat.rows(), mat.cols(), CvType.CV_8UC1); // something lol
        Core.inRange(mat, lowerOrange, upperOrange, mask);

        Core.bitwise_and(input, input, ret, mask); //something else lol
        Imgproc.GaussianBlur(mask, mask, new Size(15.0, 15.0), 0.00); //rip noise you know we had to do it to em

        //doin a contour
        ArrayList<MatOfPoint> contours = new ArrayList();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        Imgproc.drawContours(ret, contours, -1, new Scalar(0.0, 255.0, 0.0), 3); //drawing things maybe

        //finding the widest contour
        int maxWidth = 0;
        Rect maxRect =  new Rect();
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint c = contours.get(i);
            MatOfPoint2f copy =  new MatOfPoint2f(c.toArray()); //what kotlin has pointers? I gotta switch
            Rect rect = Imgproc.boundingRect(copy);

            int w = rect.width;
            // checking if the rectangle is below the horizon
            if (w > maxWidth && rect.y + rect.height > HORIZON) {
                maxWidth = w;
                maxRect = rect;
            }
            c.release(); // releasing the buffer of the contour, since after use, it is no longer needed
            copy.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
        }

        Imgproc.rectangle(ret, maxRect, new Scalar(0.0, 0.0, 255.0), 2); //drawing widest something
        Imgproc.line(ret, new Point(.0, (double) HORIZON), new Point((double) CAMERA_WIDTH, (double) HORIZON), new Scalar(255.0, .0, 255.0)); //drawing horizon

        //setting height
        if (maxWidth >= MIN_WIDTH) {
            double aspectRatio = (double) maxRect.height / (double) maxRect.width;

            /** checks if aspectRatio is greater than BOUND_RATIO
            * to determine whether stack is ONE or FOUR lol jaran comment
            */
            if (aspectRatio > BOUND_RATIO)
                height = 4;
            else
                height = 1;
        } else {
        height = 0;
        }

        telemetry.clear();
        telemetry.addData("RINGS: ", height);
        telemetry.update();

        mat.release();
        mask.release();
        hierarchy.release();

        return ret;
    }

    public int getDecision(){
        return height;
    }
}
