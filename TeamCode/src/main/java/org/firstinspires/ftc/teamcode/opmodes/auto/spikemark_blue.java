package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ochi.pipelines.TGERecognition_blue;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(group = "A", preselectTeleOp = "TeleOP")

public class spikemark_blue extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    OpenCvCamera webcam;
    TGERecognition_blue pipeline;
    TGERecognition_blue.TGEPosition snapshotAnlysis;

    SampleMecanumDrive drive;
    TrajectorySequence cazCentru;
    Pose2d startPose;
    TrajectorySequence cazStanga;
    TrajectorySequence cazDreapta;

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new TGERecognition_blue();
        webcam.setPipeline(pipeline);
        drive = new SampleMecanumDrive(hardwareMap);

        startPose = new Pose2d(0, 0, 0);
//        telemetry.addData("Cazull", snapshotAnlysis);


//
//

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);//544x288
            }

            @Override
            public void onError(int errorCode) {
            }
        });

//        telemetry.addData("Cazul",pipeline.getAnalysis());

    }
    public void init_loop(){
        telemetry.addData("Cazul",pipeline.getAnalysis());
        telemetry.update();
    }
    @Override
    public void start() {
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        snapshotAnlysis = pipeline.getAnalysis();

        switch (snapshotAnlysis) {

            case LEFT: {
                Pose2d offset = new Pose2d(12, 60, Math.toRadians(270));
                drive.setPoseEstimate(offset);

                cazStanga = drive.trajectorySequenceBuilder(startPose.plus(offset))
                        .lineToSplineHeading(new Pose2d(18  , 35, Math.toRadians(360-50)))
                        .back(5)
                        .build();
                drive.followTrajectorySequence(cazStanga);
                telemetry.addLine("Stanga");

                break;
            }

            case RIGHT: {
                Pose2d offset = new Pose2d(12, 60, Math.toRadians(270));
                drive.setPoseEstimate(offset);

                cazDreapta = drive.trajectorySequenceBuilder(startPose.plus(offset))
                        .lineToLinearHeading(new Pose2d(8.4, 35, Math.toRadians(200)))
                        .back(5)
                        .build();
                drive.followTrajectorySequence(cazDreapta);
                telemetry.addData("Cazul",snapshotAnlysis );
                break;
            }

            case CENTER: {
                Pose2d offset = new Pose2d(12, 60, Math.toRadians(270));
                drive.setPoseEstimate(offset);
                cazCentru = drive.trajectorySequenceBuilder(startPose.plus(offset))
                        .forward(27)
                        .back(5)
                        .build();
                drive.followTrajectorySequence(cazCentru);
                telemetry.addLine("Centru");
                break;
            }
//        default: {
//            drive.followTrajectorySequence(cazCentru);
//            break;
//        }

        }
    }
    @Override
    public void loop() {
        drive.update();
    }
}