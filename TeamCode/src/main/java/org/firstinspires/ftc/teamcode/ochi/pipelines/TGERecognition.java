package org.firstinspires.ftc.teamcode.ochi.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
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
    Mat blueMask = new Mat();
    Mat HSVBlurred = new Mat();

    public void inputToHSV(Mat input) {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
    }

    public Scalar TGEColorMin = new Scalar(112, 129, 77);
    public Scalar TGEColorMax = new Scalar(122, 169, 171);


    public Point rightBottomRight = new Point(500, 371);
    public Point rightTopLeft = new Point(400, 160);
    public Rect rightRect = new Rect(rightTopLeft, rightBottomRight);

    @Override
    public Mat processFrame(Mat input) {
        inputToHSV(input);
        Imgproc.rectangle(HSV, rightRect, new Scalar(0, 250, 0), 3);
        Imgproc.GaussianBlur(HSV, HSVBlurred, new Size(11, 11), 20);
        Core.inRange(HSV, TGEColorMin, TGEColorMax, blueMask);

        return blueMask;
    }
}
