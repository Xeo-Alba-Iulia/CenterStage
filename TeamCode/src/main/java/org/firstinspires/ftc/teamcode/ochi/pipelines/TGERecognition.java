package org.firstinspires.ftc.teamcode.ochi.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TGERecognition extends OpenCvPipeline {


    public enum TGEPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    Mat HSV = new Mat();
    Mat doza = new Mat();
    Mat rightTGEMat = new Mat();
    Mat leftTGEMat = new Mat();
    Mat blueMask = new Mat();
    Mat HSVBlurred = new Mat();

    public void inputToHSV(Mat input) {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
    }

    public Scalar TGEColorMin = new Scalar(82, 172, 121);
    public Scalar TGEColorMax = new Scalar(255, 255, 255);


    public Point rightBottomRight = new Point(500, 90);
    public Point rightTopLeft = new Point(540, 460);
    public Rect rightRect = new Rect(rightTopLeft, rightBottomRight);

    public Point middleBottomRight = new Point(450, 90);//trb schimbate
    public Point middleTopLeft = new Point(260, 460);//trb schimbate
    public Rect middletRect = new Rect(middleTopLeft, middleBottomRight);

    public Point leftBottomRight = new Point(60, 280);//trb schimbate
    public Point leftTopLeft = new Point(130, 460);//trb schimbate
    public Rect leftRect = new Rect(leftTopLeft, leftBottomRight);

    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(520,200);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(280,130);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(40,170);
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
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);



    Mat region1_hsv, region2_hsv, region3_hsv;
    int avg1, avg2, avg3;
    private volatile TGEPosition position = TGEPosition.LEFT;
    @Override
    public void init(Mat firstFrame)
    {
        inputToHSV(firstFrame);

        region1_hsv = HSV.submat(new Rect(region1_pointA,region1_pointB));
        region2_hsv = HSV.submat(new Rect(region2_pointA,region2_pointB));
        region3_hsv = HSV.submat(new Rect(region3_pointA,region3_pointB));
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToHSV(input);

        System.out.println("Input size: " + input.size());


        avg1 = (int) Core.mean(region1_hsv).val[0];
        avg2 = (int) Core.mean(region2_hsv).val[0];
        avg3 = (int) Core.mean(region3_hsv).val[0];

//        System.out.println("aici");

//        Imgproc.rectangle(HSV, rightRect, new Scalar(0, 0, 250), 3);
//        Imgproc.rectangle(HSV, middletRect, new Scalar(0, 0, 250), 3);
//        Imgproc.rectangle(HSV, leftRect, new Scalar(0,250, 0), 3);
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

        Imgproc.GaussianBlur(HSV, HSVBlurred, new Size(11, 11), 20,10);
        Core.inRange(HSV, TGEColorMin, TGEColorMax, blueMask);
        int maxOneTwo = Math.max(avg1, avg2);
        int max = Math.max(maxOneTwo, avg3);


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

        return HSV;
    }
    public TGEPosition getAnalysis()
    {
        return position;
    }
}
