package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ochi.pipelines.TGERecognition_Red;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class albastruAproape extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    OpenCvCamera webcam;
    TGERecognition_Red pipeline;
    TGERecognition_Red.TGEPosition snapshotAnlysis = TGERecognition_Red.TGEPosition.RIGHT;

    TrajectorySequence spikemark;
    TrajectorySequence albastru_stanga;
    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new TGERecognition_Red();
        webcam.setPipeline(pipeline);

        Pose2d startPose = new Pose2d(-62, -35, Math.toRadians(0));
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        albastru_stanga = drive.trajectorySequenceBuilder(startPose)
//                .splineToLinearHeading(new Pose2d(31, 10, Math.toRadians(0)), Math.toRadians(0))
//                .back(10)
//                .lineToLinearHeading(new Pose2d(18, 46.1, Math.toRadians(-90)))
                .build();

//        pipeline.processFrame();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            snapshotAnlysis = pipeline.getAnalysis();
            telemetry.addLine(snapshotAnlysis.toString());
            switch (snapshotAnlysis) {

                case LEFT: {
                    drive.followTrajectorySequence(albastru_stanga);
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
                default:{
                    telemetry.addLine("ceva");
                }
            }

        }
    }
}