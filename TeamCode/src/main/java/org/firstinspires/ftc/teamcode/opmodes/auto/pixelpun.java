package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ochi.pipelines.TGERecognition;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous
public class pixelpun extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    OpenCvCamera webcam;
    TGERecognition pipeline;
    TGERecognition.TGEPosition snapshotAnlysis = TGERecognition.TGEPosition.RIGHT;


    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new TGERecognition();
        webcam.setPipeline(pipeline);
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            snapshotAnlysis = pipeline.getAnalysis();

            switch (snapshotAnlysis) {
                case LEFT: {
                    telemetry.addLine("merge pe stanga");
                    break;
                }

                case RIGHT: {
                    telemetry.addLine("merge pe dreapta");
                    break;
                }

                case CENTER: {
                    telemetry.addLine("merge numa ca pe mijloc");
                    break;
                }
            }

        }
    }
}