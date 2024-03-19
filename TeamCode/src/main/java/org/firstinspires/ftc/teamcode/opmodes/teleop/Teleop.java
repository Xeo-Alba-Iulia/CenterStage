package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robothardware;
import org.firstinspires.ftc.teamcode.sisteme.Ridicare;
import org.firstinspires.ftc.teamcode.sisteme.Spanzurare;
import org.firstinspires.ftc.teamcode.utilities.PoseStorage;
import org.firstinspires.ftc.teamcode.utilities.ServoSmoothing;


@TeleOp(name = "TeleOP", group = "A")

public class Teleop extends OpMode {

    robothardware robot = new robothardware(this);
    SampleMecanumDrive drive;

    //pozitie auxiliara servo smooth
    private double midPos;

    //Vector poz avion
    Vector2d VectorAvion;
    Vector2d constantheading;
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
    private Pose2d targetPosition = new Pose2d(0, 0,Math.toRadians(180));

    Spanzurare.HangingState currentHang;

    enum driverMode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }
    enum ServoPos{
        IN_PROGRESS,
        IDLE
    }
    enum HeadingState{
        VARIABLE,
        CONSTANT
    }

    enum LiftState{
        INTAKE,
        UP1,
        UP2,
        UP3,
        DOWN1,
        DOWN2
    }
    enum PendulState{
        AUTO,
        MANUAL
    }
    LiftState currentLiftState = LiftState.INTAKE;
    ServoPos currentServoPos = ServoPos.IDLE;
    driverMode currentMode = driverMode.DRIVER_CONTROL;
    PendulState currentPend = PendulState.AUTO;
    HeadingState currentHeading = HeadingState.VARIABLE;




    @Override
    public void init() {
        robot.init();
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(PoseStorage.currentPose);
        VectorAvion = new Vector2d(47, 13);
        constantheading = new Vector2d(gamepad1.left_stick_x,gamepad1.left_stick_y);
        robot.pendulare.setPosition(robot.pendul_intake);
        robot.aligner.setPosition(robot.aligner_intake);
        robot.usa.setPosition(robot.door.usa_intake);
        robot.plane.setPosition(robot.avion.avion_armat);
        headingController.setInputBounds(-Math.PI, Math.PI);
        

    }

    public void loop() {

        Pose2d poseEstimate = drive.getPoseEstimate();
        Pose2d driveDirection = new Pose2d();

        switch (currentMode){
            case DRIVER_CONTROL:
                switch (currentHeading){
                    case VARIABLE:
                        driveDirection = new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        );
                        if (gamepad1.left_stick_button) {
                            currentHeading = HeadingState.CONSTANT;
                        }
                        break;
                    case CONSTANT:

                        Vector2d gamepadInput = new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        );
                        headingController.setTargetPosition(360-Math.abs(PoseStorage.currentPose.getHeading()));

                        double headingInput = (headingController.update(poseEstimate.getHeading())
                                * DriveConstants.kV)
                                * DriveConstants.TRACK_WIDTH;


                        driveDirection = new Pose2d(
                                gamepadInput,
                                headingInput
                        );

                            if(gamepad1.right_stick_button) {
                                currentHeading = HeadingState.VARIABLE;
                            }
                            if(gamepad1.right_bumper)
                                currentHeading = HeadingState.VARIABLE;

                }
                drive.setWeightedDrivePower(driveDirection);
                robot.avion.PlaneLaunching(gamepad1);
                robot.door.doorOpening(gamepad1);
                robot.intake.runIntake(gamepad2);
                robot.hanging.goToPosHanging(gamepad2);

                switch (currentLiftState){
                    case INTAKE:
                        robot.lift.target = Ridicare.POS_1;
                        robot.aligner.setPosition(robot.aligner_intake);
                        if (gamepad2.dpad_left){
                            currentLiftState = LiftState.UP1;
                            currentPend= PendulState.AUTO;
                        }
                        else if (gamepad2.dpad_up){
                            currentLiftState = LiftState.UP2;
                            currentPend= PendulState.AUTO;
                        }
                        else if (gamepad2.dpad_right){
                            currentLiftState = LiftState.UP3;
                            currentPend= PendulState.AUTO;
                        }
                        break;
                    case UP1:
                        robot.lift.target=Ridicare.POS_2;
                        switch (currentPend){
                            case AUTO:
                                robot.pendulare.setPosition(robot.pendul_outtake);

                                if(gamepad2.left_stick_button)
                                    currentPend = PendulState.MANUAL;
                            case MANUAL:
                                robot.pendulManual.MiscarePendManuala(gamepad2);
                                if(gamepad2.right_stick_button)
                                    currentPend = PendulState.AUTO;
                        }
                        robot.aligner.setPosition(robot.aligner_outake);
                        if (gamepad2.dpad_down){
                            currentLiftState = LiftState.DOWN1;
                            currentServoPos = ServoPos.IN_PROGRESS;
                            currentPend= PendulState.AUTO;

                        } else if (gamepad2.dpad_up){
                            currentLiftState = LiftState.UP2;
                            currentPend= PendulState.AUTO;
                        }
                        else if (gamepad2.dpad_right){
                            currentLiftState = LiftState.UP3;
                            currentPend= PendulState.AUTO;
                        }
                        break;
                    case UP2:
                        robot.lift.target=Ridicare.POS_3;//trb masurat cu encoderu
                        switch (currentPend){
                            case AUTO:
                                robot.pendulare.setPosition(robot.pendul_outtake);
                                if(gamepad2.left_stick_button)
                                    currentPend = PendulState.MANUAL;
                            case MANUAL:
                                robot.pendulManual.MiscarePendManuala(gamepad2);
                                if(gamepad2.right_stick_button)
                                    currentPend = PendulState.AUTO;
                        }
                        robot.aligner.setPosition(robot.aligner_outake);
                        if (gamepad2.dpad_down){
                            currentLiftState = LiftState.DOWN1;
                            currentServoPos = ServoPos.IN_PROGRESS;
                        }
                        else if (gamepad2.dpad_left){
                            currentLiftState = LiftState.UP1;
                            currentPend= PendulState.AUTO;
                        }
                        else if (gamepad2.dpad_right){
                            currentLiftState = LiftState.UP3;
                            currentPend= PendulState.AUTO;
                        }
                        break;
                    case UP3:
                        robot.lift.target=Ridicare.POS_4;//trb masurat cu encoderu
                        switch (currentPend){
                            case AUTO:
                                robot.pendulare.setPosition(robot.pendul_outtake);
                                if(gamepad2.left_stick_button)
                                    currentPend = PendulState.MANUAL;
                            case MANUAL:
                                robot.pendulManual.MiscarePendManuala(gamepad2);
                                if(gamepad2.right_stick_button)
                                    currentPend = PendulState.AUTO;
                        }                        robot.aligner.setPosition(robot.aligner_outake);

                        if (gamepad2.dpad_down){
                            currentLiftState = LiftState.DOWN1;
                            currentServoPos = ServoPos.IN_PROGRESS;
                        }
                        else if(gamepad2.dpad_left) {
                            currentLiftState = LiftState.UP1;
                            currentPend = PendulState.AUTO;
                        }
                        else if (gamepad2.dpad_up) {
                        currentLiftState = LiftState.UP2;
                        currentPend = PendulState.AUTO;
                    }
                        break;
                    case DOWN1:
                        robot.lift.target = Ridicare.POS_1;
                        robot.aligner.setPosition(robot.aligner_intake);
                        robot.pendulare.setPosition(0.35);
                        if(robot.lift.getCurrentPosition()<50) {
                            currentLiftState = LiftState.DOWN2;
                            currentPend= PendulState.AUTO;
                            midPos = robot.pendulare.getPosition();
                        }
                        break;
                    case DOWN2:

                        switch (currentServoPos){
                            case IDLE:
                            robot.pendulare.getPosition();
                                break;
                            case IN_PROGRESS:
                                robot.pendulare.setPosition(ServoSmoothing.servoSmoothing(midPos, robot.pendul_intake));
                                if(Math.abs(robot.pendulare.getPosition() - robot.pendul_intake)<0.005) {
                                    robot.pendulare.setPosition(robot.pendul_intake);
                                    currentServoPos = ServoPos.IDLE;
                                    currentLiftState = LiftState.INTAKE;
                                }
                                else {
                                    midPos = robot.pendulare.getPosition();
                                }
                                break;
                        }
                        break;
                }
                    if (gamepad1.y) {
                        Trajectory avion = drive.trajectoryBuilder(poseEstimate)
                                .splineToLinearHeading(new Pose2d(47,13), Math.toRadians(0))
                                .build();
                        drive.followTrajectoryAsync(avion);
                        currentMode = driverMode.AUTOMATIC_CONTROL;
                    }

                    break;
                case AUTOMATIC_CONTROL:
                    if(gamepad1.x) {
                        drive.breakFollowing();
                        currentMode = driverMode.DRIVER_CONTROL;
                    }
                    if (!drive.isBusy()) {
                        currentMode = driverMode.DRIVER_CONTROL;
                    }
                    break;
            }
        headingController.update(poseEstimate.getHeading());
        drive.update();
        robot.lift.update();
        telemetry.addData("Case Driver Control: ", currentMode);
        telemetry.addData("Case Sisteme: ", currentLiftState);
        telemetry.addData("Case Servo", currentServoPos);
        telemetry.addData("Case Pendul Manual",currentPend);
        telemetry.addData("Pozitie Hang",robot.spanzurare.getCurrentPosition());
        telemetry.addData("Poitie Lift", robot.lift.getCurrentPosition());
        telemetry.addData("Poitie Pendul", robot.pendulare.getPosition());
        telemetry.addData("Poitie MidPos", midPos);
        telemetry.addData("Caz hang",currentHang);
        telemetry.addData("Heading", poseEstimate.getHeading());
        telemetry.update();

    }


}
