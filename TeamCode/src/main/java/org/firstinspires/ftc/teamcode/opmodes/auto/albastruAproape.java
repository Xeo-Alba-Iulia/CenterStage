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
import org.firstinspires.ftc.teamcode.utilities.ServoSmoothing;
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
    private double midPos;



    int caz = 0;
    Trajectory spikemark;
    Trajectory spikemarkspate;
    Trajectory heading_align;
    Trajectory backdrop_align;
    Trajectory backdrop_fata;
    Trajectory parcare_align;
    Trajectory parcare;

    enum caseState{
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
        SPATE_SPIKEMARK_SCURT,
        SPATE_SPIKEMARK_LUNG,
        HEADING_ALIGN,
        BACKDROP_ALIGN,
        PARCARE_ALIGN,
        PARCARE,
        FATA_BACKDROP,
        IDLE
    }
    Pose2d startPose = new Pose2d(12, 60, Math.toRadians(270));



    caseState cazState = caseState.STANGA;
    trajState currentState = trajState.SPIKEMARK;
    ServoPos currentServoPos = ServoPos.IDLE;

    robothardware robot;


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
                .lineToLinearHeading(new Pose2d(16, 35, Math.toRadians(270+40)))
                .build();
        spikemarkspate = drive.trajectoryBuilder(spikemark.end())
                .back(5)
                .build();

        heading_align = drive.trajectoryBuilder(spikemarkspate.end())
                .lineToLinearHeading(new Pose2d(24,50,Math.toRadians(180)))
                .build();
        backdrop_align = drive.trajectoryBuilder(heading_align.end())
                .lineToLinearHeading(new Pose2d(46,35,Math.toRadians(180)))
                .build();
        backdrop_fata = drive.trajectoryBuilder(backdrop_align.end())
                .forward(15)
                .build();

        parcare_align = drive.trajectoryBuilder(backdrop_fata.end())
                .strafeRight(24)
                .build();
        parcare = drive.trajectoryBuilder(parcare_align.end())
                .back(2)
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
        snapshotAnlysis = pipeline.getAnalysis();
        switch (snapshotAnlysis) {

            case LEFT: {
//                cazState = caseState.STANGA;
//                telemetry.addLine("stanga");
                break;
            }

            case RIGHT: {

//            telemetry.addLine("Dreapta");
//                cazState = caseState.DREAPTA;

                spikemark= drive.trajectoryBuilder(startPose)
                        .lineToLinearHeading(new Pose2d(7, 30, Math.toRadians(200)))
                        .build();
                spikemarkspate = drive.trajectoryBuilder(spikemark.end())
                        .back(5)
                        .build();
                backdrop_align = drive.trajectoryBuilder(heading_align.end())
                        .lineToLinearHeading(new Pose2d(46,25,Math.toRadians(180)))
                        .build();
                break;
            }

            case CENTER: {
//                cazState = caseState.MIJLOC;
//                telemetry.addLine("mij");
                spikemark = drive.trajectoryBuilder(startPose)
                        .lineToLinearHeading(new Pose2d(12, 29, Math.toRadians(270)))
                        .build();
                spikemarkspate = drive.trajectoryBuilder(spikemark.end())
                        .back(5)
                        .build();
                backdrop_align = drive.trajectoryBuilder(heading_align.end())
                        .lineToLinearHeading(new Pose2d(46,28,Math.toRadians(180)))
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
        drive.followTrajectoryAsync(spikemark);

    }
    @Override
    public void loop() {

        switch (currentState) {
                    case SPIKEMARK:
//                robot.lift.target = 200;
//                robot.usa.setPosition(0.1);
                        if (!drive.isBusy()) {
                            robot.intec.setPower(-0.4);
                            currentState = trajState.SPATE_SPIKEMARK_SCURT;
                            drive.followTrajectoryAsync(spikemarkspate);
                            break;
                        }
                    case SPATE_SPIKEMARK_SCURT:
                        if (!drive.isBusy()) {
                            robot.intec.setPower(0);
                            currentState = trajState.HEADING_ALIGN;
                            drive.followTrajectoryAsync(heading_align);
                        }
                        break;
                    case HEADING_ALIGN:
                        if (!drive.isBusy()) {
                            robot.lift.target = Ridicare.POS_2;
                            robot.pendulare.setPosition(robot.pendul_outtake);
                            robot.aligner.setPosition(robot.aligner_outake);
                            currentState = trajState.BACKDROP_ALIGN;
                            drive.followTrajectoryAsync(backdrop_align);
                            currentServoPos = ServoPos.IN_PROGRESS;

                        }
                        break;
                    case BACKDROP_ALIGN:

                        if (!drive.isBusy()) {
                            robot.usa.setPosition(robot.door.usa_outtake);
                            currentState= trajState.FATA_BACKDROP;
                            drive.followTrajectoryAsync(backdrop_fata);
                            midPos = robot.pendulare.getPosition();


                        }

                        break;
                    case FATA_BACKDROP:

                        if(!drive.isBusy()) {

//                            switch (currentServoPos){
//
//                                case IDLE:
////                                    midPos= robot.pendulare.getPosition();
//                                    break;
//                                case IN_PROGRESS:
//                                    robot.pendulare.setPosition(ServoSmoothing.servoSmoothing(midPos, robot.pendul_intake));
//                                    if(Math.abs(robot.pendulare.getPosition() - robot.pendul_intake)<0.005) {
//                                        robot.pendulare.setPosition(robot.pendul_intake);
//                                        currentServoPos = ServoPos.IDLE;
//                                    }
//                                    else {
//                                        midPos = robot.pendulare.getPosition();
//                                    }
//                                    break;
//                            }

                            drive.followTrajectoryAsync(parcare_align);
                            currentState= trajState.PARCARE_ALIGN;
                        }
                        switch (currentServoPos){

                            case IDLE:
//                                    midPos= robot.pendulare.getPosition();
                                break;
                            case IN_PROGRESS:
                                robot.usa.setPosition(robot.door.usa_intake);
                                robot.aligner.setPosition(robot.aligner_intake);

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

                        break;
                    case PARCARE_ALIGN:
                        if (!drive.isBusy()) {
                            currentState = trajState.PARCARE;
                            drive.followTrajectoryAsync(parcare);
//                            drive.followTrajectoryAsync(parcare); w
                        }
                        break;
                    case PARCARE:
                        if(drive.isBusy()) {
                            currentState = trajState.IDLE;
                        }
                         break;
                    case IDLE:
                        break;
                }
        drive.update();
                telemetry.addData("CaseTraj",currentState);
//                telemetry.addData("Traj",drive.)
        PoseStorage.currentPose = drive.getPoseEstimate();

    }

}