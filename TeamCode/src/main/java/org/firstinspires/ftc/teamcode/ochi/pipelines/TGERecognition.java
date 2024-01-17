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

    public Scalar TGEColorMin = new Scalar(73, 136, 121);
    public Scalar TGEColorMax = new Scalar(126, 255, 255);


    public Point rightBottomRight = new Point(450, 90);
    public Point rightTopLeft = new Point(260, 460);
    public Rect rightRect = new Rect(rightTopLeft, rightBottomRight);

    public Point middleBottomRight = new Point(450, 90);//trb schimbate
    public Point middleTopLeft = new Point(260, 460);//trb schimbate
    public Rect middletRect = new Rect(middleTopLeft, middleBottomRight);

    public Point leftBottomRight = new Point(450, 90);//trb schimbate
    public Point leftTopLeft = new Point(260, 460);//trb schimbate
    public Rect leftRect = new Rect(leftTopLeft, leftBottomRight);


    Mat region1_hsv, region2_hsv, region3_hsv;
    int avg1, avg2, avg3;
    private volatile TGEPosition position = TGEPosition.LEFT;
    @Override
    public void init(Mat firstFrame)
    {
        inputToHSV(firstFrame);

        region1_hsv = HSV.submat(rightRect);
        region2_hsv = HSV.submat(middletRect);
        region3_hsv = HSV.submat(leftRect);
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToHSV(input);

        avg1 = (int) Core.mean(region1_hsv).val[0];
        avg2 = (int) Core.mean(region2_hsv).val[0];
        avg3 = (int) Core.mean(region3_hsv).val[0];

        Imgproc.rectangle(HSV, rightRect, new Scalar(0, 250, 0), 3);
        Imgproc.rectangle(HSV, middletRect, new Scalar(0, 250, 0), 3);
        Imgproc.rectangle(HSV, leftRect, new Scalar(0, 250, 0), 3);
        Imgproc.GaussianBlur(HSV, HSVBlurred, new Size(11, 11), 20,10);
        Core.inRange(HSV, TGEColorMin, TGEColorMax, blueMask);
        int maxOneTwo = Math.max(avg1, avg2);
        int max = Math.max(maxOneTwo, avg3);


        if(max == avg1)
        {
            position = TGEPosition.RIGHT;


            Imgproc.rectangle(HSV, rightRect,new Scalar(0,250,0),3);
        }
        else if(max == avg2)
        {
            position = TGEPosition.CENTER;


            Imgproc.rectangle(HSV, middletRect,new Scalar(0,250,0),3);
        }
        else if(max == avg3)
        {
            position = TGEPosition.LEFT;

            Imgproc.rectangle(HSV, leftRect,new Scalar(0,250,0),3);
        }

        return HSV;
    }
    public TGEPosition getAnalysis()
    {
        return position;
    }
}
