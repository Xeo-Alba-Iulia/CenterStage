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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utilities.PoseStorage;
import org.firstinspires.ftc.teamcode.utilities.ServoSmoothing;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(group = "A", preselectTeleOp = "TeleOP")

public class albastruDeparte extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    OpenCvCamera webcam;
    TGERecognition_blue pipeline;
    TGERecognition_blue.TGEPosition snapshotAnlysis;

    protected SampleMecanumDrive drive;
    private double midPos;



    int caz = 0;
    protected Trajectory spikemark, spikemarkspate, stagedoor1, stagedoor2,
            backdrop_align, backdrop_fata, park_align, park;

    protected TrajectorySequence heading_align;

    protected enum caseState{
        STANGA,
        MIJLOC,
        DREAPTA
    }
    enum ServoPos{
        IN_PROGRESS,
        IDLE
    }


    enum trajState {
        SPIKEMARK,
        SPIKEMARK_SPATE,
        HEADING_ALIGN,
        STAGEDOOR1,
        STAGEDOOR2,
        BACKDROP_ALIGN,
        FATA_BACKDROP,
        PARK_ALIGN,
        PARK,
        IDLE
    }
    protected Pose2d startPose = new Pose2d(-38,60,Math.toRadians(270));



    caseState cazState = caseState.STANGA;
    trajState currentState = trajState.SPIKEMARK;
    ServoPos currentServoPos = ServoPos.IDLE;

    protected robothardware robot;


    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new TGERecognition_blue();
        webcam.setPipeline(pipeline);
        robot = new robothardware(this);
        robot.init();
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        robot.pendulare.setPosition(robot.pendul_intake);
        robot.aligner.setPosition(robot.aligner_intake);
        robot.usa.setPosition(robot.door.usa_intake);

        spikemark = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-32, 30, Math.toRadians(360-30)))
                .build();
        spikemarkspate = drive.trajectoryBuilder(spikemark.end())
                .back(5)
                .build();
        heading_align = drive.trajectorySequenceBuilder(spikemarkspate.end())
                .lineToLinearHeading(new Pose2d(-46, 10, Math.toRadians(180)))
                .build();
        stagedoor1 = drive.trajectoryBuilder(heading_align.end())
                .back(80)
                .build();
        stagedoor2 =drive.trajectoryBuilder(stagedoor1.end())
                .back(5)
                .build();
        backdrop_align = drive.trajectoryBuilder(stagedoor2.end())
                .lineToLinearHeading(new Pose2d(55, 51, Math.toRadians(180)))
                .build();
        backdrop_fata = drive.trajectoryBuilder(backdrop_align.end())
                .forward(15)
                .build();
        park_align = drive.trajectoryBuilder(backdrop_fata.end())
                .strafeLeft(45)
                .build();
        park = drive.trajectoryBuilder(park_align.end())
                .back(15)
                .build();


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
        snapshotAnlysis = pipeline.getAnalysis();
        switch (snapshotAnlysis) {

            case LEFT: {
                cazState = caseState.STANGA;
                telemetry.addLine("stanga");
                break;
            }

            case RIGHT: {

                telemetry.addLine("Dreapta");
                cazState = caseState.DREAPTA;

                spikemark= drive.trajectoryBuilder(startPose)
                        .lineToLinearHeading(new Pose2d(-40, 35, Math.toRadians(270-40)))
                        .build();
                spikemarkspate = drive.trajectoryBuilder(spikemark.end())
                        .lineToLinearHeading(new Pose2d(-34,38,Math.toRadians(180)))
                        .build();
                heading_align = drive.trajectorySequenceBuilder(spikemarkspate.end())
                        .lineToLinearHeading(new Pose2d(-35, 10, Math.toRadians(180)))
                        .build();
                break;
            }

            case CENTER: {
                cazState = caseState.MIJLOC;
                telemetry.addLine("mij");
                spikemark= drive.trajectoryBuilder(startPose)
                        .lineToLinearHeading(new Pose2d(-33,32,Math.toRadians(270+20)))
                        .build();
                spikemarkspate = drive.trajectoryBuilder(spikemark.end())
                        .back(5)
                        .build();
                heading_align = drive.trajectorySequenceBuilder(spikemarkspate.end())
                        .lineToLinearHeading(new Pose2d(-45,33,Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(-45, 10, Math.toRadians(180)))
                        .build();

                break;
            }
        }
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
    }
    @Override
    public void loop() {
        switch (currentState) {
            case SPIKEMARK:
//                robot.lift.target = 200;
//                robot.usa.setPosition(0.1);
                if (!drive.isBusy()) {
                    robot.intec.setPower(-0.55);

                    currentState = trajState.SPIKEMARK_SPATE;
                    drive.followTrajectoryAsync(spikemarkspate);
                    break;
                }
            case SPIKEMARK_SPATE:

                if (!drive.isBusy()) {

                    currentState = trajState.HEADING_ALIGN;
                    drive.followTrajectorySequenceAsync(heading_align);
                }
                break;
            case HEADING_ALIGN:
                if (!drive.isBusy()) {
                    robot.intec.setPower(0);
                    currentState = trajState.STAGEDOOR1;
                    drive.followTrajectoryAsync(stagedoor1);
                }
                break;
            case STAGEDOOR1:
                if(!drive.isBusy()){
                    currentState = trajState.STAGEDOOR2;
                    drive.followTrajectoryAsync(stagedoor2);
                }
            case STAGEDOOR2:
                if(!drive.isBusy()){
                    robot.pendulare.setPosition(0.7);
                    robot.aligner.setPosition(robot.aligner_outake);
                    currentState=trajState.BACKDROP_ALIGN;
                    drive.followTrajectoryAsync(backdrop_align);
                }

            case BACKDROP_ALIGN:

                if (!drive.isBusy()) {
                    robot.usa.setPosition(robot.door.usa_outtake);
                    currentState= trajState.FATA_BACKDROP;
                    drive.followTrajectoryAsync(backdrop_fata);
                    currentServoPos = ServoPos.IN_PROGRESS;
                }
                midPos = robot.pendulare.getPosition();

                break;
            case FATA_BACKDROP:
                if(!drive.isBusy()) {
                    switch (currentServoPos){

                        case IDLE:
                            midPos= robot.pendulare.getPosition();
                            break;
                        case IN_PROGRESS:
                            robot.pendulare.setPosition(ServoSmoothing.servoSmoothing(midPos, robot.pendul_intake));
                            if(Math.abs(robot.pendulare.getPosition() - robot.pendul_intake)<0.005) {
                                robot.pendulare.setPosition(robot.pendul_intake);
                                currentServoPos = ServoPos.IDLE;
                            }
                            else {
                                midPos = robot.pendulare.getPosition();
                            }
                            break;
                    }
                    robot.usa.setPosition(robot.door.usa_intake);
                    robot.aligner.setPosition(robot.aligner_intake);
                    drive.followTrajectoryAsync(park_align);
                    currentState= trajState.PARK_ALIGN;
                }
                break;
            case PARK_ALIGN:
                if (!drive.isBusy()) {
                    currentState = trajState.PARK;
                    drive.followTrajectoryAsync(park);
//                            drive.followTrajectoryAsync(parcare); w
                }
                break;
            case PARK:
                if(drive.isBusy()) {
                    currentState = trajState.IDLE;
                }
                break;
            case IDLE:
                break;
        }
        drive.update();
        telemetry.addData("CaseTraj",currentState);
        telemetry.addData("servo smooth",currentServoPos);
//                telemetry.addData("Traj",drive.)
        PoseStorage.currentPose = drive.getPoseEstimate();

    }

}