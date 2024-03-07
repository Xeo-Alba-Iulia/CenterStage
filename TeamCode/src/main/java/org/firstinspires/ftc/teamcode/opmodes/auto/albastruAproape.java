package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ochi.pipelines.TGERecognition_blue;
import org.firstinspires.ftc.teamcode.robothardware;
import org.firstinspires.ftc.teamcode.sisteme.Ridicare;
import org.firstinspires.ftc.teamcode.utilities.PoseStorage;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(group = "A", preselectTeleOp = "TeleOP")

public class albastruAproape extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    OpenCvCamera webcam;
    TGERecognition_blue pipeline;
    TGERecognition_blue.TGEPosition snapshotAnlysis;

    SampleMecanumDrive drive;

    int caz = 0;

    enum trajState {
        SPIKEMARK,
        SPATE_SPIKEMARK_SCURT,
        SPATE_SPIKEMARK_LUNG,
        HEADING_ALIGN,
        BACKDROP_ALIGN,
        PARCARE_ALIGN,
        PARCARE,
        IDLE
    }
    Pose2d startPose = new Pose2d(12, 60, Math.toRadians(270));

    Trajectory spikemark = drive.trajectoryBuilder(startPose)
            .lineToLinearHeading(new Pose2d(13, 35, Math.toRadians(360-30)))
            .build();
    Trajectory spikemarkspate = drive.trajectoryBuilder(spikemark.end())
            .back(3)
            .build();

    Trajectory heading_align = drive.trajectoryBuilder(spikemarkspate.end())
            .lineToLinearHeading(new Pose2d(24,60,Math.toRadians(180)))
            .build();
    Trajectory backdrop_align = drive.trajectoryBuilder(heading_align.end())
            .lineToLinearHeading(new Pose2d(46,40,Math.toRadians(180)))
            .build();
    Trajectory parcare_align = drive.trajectoryBuilder(backdrop_align.end())
            .strafeRight(19)
            .build();
    Trajectory parcare = drive.trajectoryBuilder(parcare_align.end())
            .build();

    trajState currentState = trajState.SPIKEMARK;
    robothardware robot = new robothardware(this);


    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new TGERecognition_blue();
        webcam.setPipeline(pipeline);
        drive = new SampleMecanumDrive(hardwareMap);
        robot.init();



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
        drive.followTrajectoryAsync(spikemark);
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
                telemetry.addLine("Stanga");

                switch (currentState) {
                    case SPIKEMARK:
                        if(!drive.isBusy()){
                            currentState= trajState.SPATE_SPIKEMARK_SCURT;
                            drive.followTrajectoryAsync(spikemarkspate);
                        }
                        break;
                    case SPATE_SPIKEMARK_SCURT:
                        if(!drive.isBusy()){
                            currentState= trajState.HEADING_ALIGN;
                            drive.followTrajectoryAsync(heading_align);
                        }
                    case HEADING_ALIGN:
                        if(!drive.isBusy()){
                            currentState= trajState.BACKDROP_ALIGN;
                            drive.followTrajectoryAsync(backdrop_align);
                        }
                    case BACKDROP_ALIGN:
                        robot.lift.target = Ridicare.POS_2;
                        robot.pendulare.setPosition(robot.pendul_outtake);
                        robot.aligner.setPosition(robot.aligner_outake);
                        if(!drive.isBusy()){
                            robot.usa.setPosition(robot.door.usa_outtake);
                            currentState= trajState.PARCARE_ALIGN;
                            drive.followTrajectoryAsync(parcare_align);
                        }
                    case PARCARE_ALIGN:
                        if(!drive.isBusy()){
                            currentState= trajState.PARCARE;
                            drive.followTrajectoryAsync(parcare);
                        }
                    case PARCARE:
                        if(!drive.isBusy()){
                            currentState= trajState.IDLE;
                        }
                }

                break;
            }

            case RIGHT: {
                Pose2d offset = new Pose2d(12, 60, Math.toRadians(270));
                drive.setPoseEstimate(offset);


                telemetry.addData("Cazul",snapshotAnlysis );
                break;
            }

            case CENTER: {
                Pose2d offset = new Pose2d(12, 60, Math.toRadians(270));
                drive.setPoseEstimate(offset);

                telemetry.addLine("Centru");
                break;
            }
        }
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
    @Override
    public void loop() {
        drive.update();
    }

}