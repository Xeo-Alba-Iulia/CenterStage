package org.firstinspires.ftc.teamcode.ochi.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TGERecognition_Red extends OpenCvPipeline {


    public TGEPosition getAnalysis()
    {
        return position;
    }




    public enum TGEPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    Telemetry telemetry;

    public TGERecognition_Red(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public TGERecognition_Red() {
    }


    Mat HSV = new Mat();
    Mat doza = new Mat();
    Mat rightTGEMat = new Mat();
    Mat leftTGEMat = new Mat();
    Mat redMask = new Mat();
    Mat HSVBlurred = new Mat();

    public void inputToHSV(Mat input) {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
    }

    public Scalar TGEColorMin = new Scalar(1, 110, 159);
    public Scalar TGEColorMax = new Scalar(21, 238, 255);




    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(520,200);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(280,130);
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(40,170);
    static final int REGION_WIDTH = 110;
    static final int REGION_HEIGHT = 200;
    static final Scalar GREEN = new Scalar(0, 255, 0);

    /*
     * Points which actually define the sample region rectangles, derived from above values
     *
     * Example of how points A and B work to define a rectangle
     *
     *   ------------------------------------
     *   | (0,0) Point A                    |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                  Point B (70,50) |
     *   ------------------------------------
     *
     */
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + (double) REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + (double) REGION_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + (double) REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + (double) REGION_HEIGHT);
    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + (double) REGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + (double) REGION_HEIGHT);



    Mat region1_hsv, region2_hsv, region3_hsv;
    double avg1, avg2, avg3;
    public volatile TGEPosition position = TGEPosition.LEFT;
    @Override
    public void init(Mat firstFrame)
    {
        inputToHSV(firstFrame);


//        telemetry.addData("abc",HSV.cols());
//        telemetry.addData("def", HSV.rows());
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToHSV(input);
        Core.flip(HSV,HSV,0);
        Core.flip(HSV,HSV,1);

        Imgproc.rectangle(
                HSV, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 2 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                HSV, // Buffer to draw on
                region2_pointA, // First point which defines the rectangle
                region2_pointB, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 3 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                HSV, // Buffer to draw on
                region3_pointA, // First point which defines the rectangle
                region3_pointB, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        Imgproc.GaussianBlur(HSV, HSVBlurred, new Size(11, 11), 30,30);
        Core.inRange(HSV, TGEColorMin, TGEColorMax, redMask);
        double max = Math.max(Math.max(avg1, avg2), avg3);

        region1_hsv = redMask.submat(new Rect(region1_pointA,region1_pointB));
        region2_hsv = redMask.submat(new Rect(region2_pointA,region2_pointB));
        region3_hsv = redMask.submat(new Rect(region3_pointA,region3_pointB));

        System.out.println("Input size: " + input.size());


        avg1 = Core.mean(region1_hsv).val[0];
        avg2 = Core.mean(region2_hsv).val[0];
        avg3 = Core.mean(region3_hsv).val[0];


        if (telemetry != null) {
            telemetry.addData("size", redMask.rows());
            telemetry.addData("size col", redMask.cols());
            telemetry.addData("size", HSV.rows());
            telemetry.addData("size col", HSV.cols());
            telemetry.addData("AVERAGE 1", Core.mean(region1_hsv).val[0]);
            telemetry.addData("AVERAGE 2", Core.mean(region2_hsv).val[0]);
            telemetry.addData("AVERAGE 3", Core.mean(region3_hsv).val[0]);
            telemetry.update();
        }


//        System.out.println("aici");

//        Imgproc.rectangle(HSV, rightRect, new Scalar(0, 0, 250), 3);
//        Imgproc.rectangle(HSV, middletRect, new Scalar(0, 0, 250), 3);
//        Imgproc.rectangle(HSV, leftRect, new Scalar(0,250, 0), 3);
        Core.flip(redMask,redMask,1);
        Core.flip(redMask,redMask,1);
//

        if(max == avg1)
        {
            position = TGEPosition.LEFT;


            Imgproc.rectangle(HSV, region1_pointA,region1_pointB,new Scalar(250,0,0),3);
        }
        else if(max == avg2)
        {
            position = TGEPosition.CENTER;


            Imgproc.rectangle(HSV, region2_pointA,region2_pointB,new Scalar(250,0,0),3);
        }
        else if(max == avg3)
        {
            position = TGEPosition.RIGHT;

            Imgproc.rectangle(HSV, region3_pointA,region3_pointB,new Scalar(250,0,0),3);
        }

        return redMask;
    }

}
