package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robothardware;
import org.firstinspires.ftc.teamcode.sisteme.Ridicare;
import org.firstinspires.ftc.teamcode.utilities.DriveTeleOP;
import org.firstinspires.ftc.teamcode.utilities.PoseStorage;
import org.firstinspires.ftc.teamcode.utilities.ServoSmoothing;


@TeleOp(name = "TeleOP", group = "A")

public class Teleop extends OpMode {

    robothardware robot = new robothardware(this);
    DriveTeleOP drive;

    //pozitie auxiliara servo smooth
    private double midPos;

    //Vector poz avion
    Vector2d VectorAvion;

    //State machine go_to_point in teleop
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    //State machine servo smooth
    enum ServoPos{
        IN_PROGRESS,
        IDLE
    }

    //State machine automatizati sisteme
    enum LiftState{
        INTAKE,
        UP1,
        UP2,
        UP3,
        DOWN1,
        DOWN2
    }
    LiftState currentLiftState = LiftState.INTAKE;
    ServoPos currentServoPos = ServoPos.IDLE;
    Mode currentMode = Mode.DRIVER_CONTROL;

    @Override
    public void init() {
        robot.init();
        drive = new DriveTeleOP(hardwareMap);
        drive.setPoseEstimate(PoseStorage.currentPose);
        VectorAvion = new Vector2d(47, 13);
        robot.pendulare.setPosition(robot.pendul_intake);
        robot.aligner.setPosition(robot.aligner_intake);
        robot.usa.setPosition(robot.door.usa_intake);
//        robot.plane.setPosition(robot.avion_armat);
    }

    public void loop() {

        Pose2d poseEstimate = drive.getPoseEstimate();

        switch (currentMode){
            case DRIVER_CONTROL:
                robot.movement(gamepad1);
                robot.avion.PlaneLaunching(gamepad1);
                robot.door.doorOpening(gamepad1);
                robot.intake.runIntake(gamepad2);
                robot.hanging.goToPosHanging(gamepad2);
                robot.pendulManual.MiscarePendManuala(gamepad2);
                switch (currentLiftState){
                    case INTAKE:
                        robot.lift.target = 10 /*Ridicare.POS_1*/;
                        robot.aligner.setPosition(robot.aligner_intake);
                        if (gamepad2.dpad_left)
                            currentLiftState = LiftState.UP1;
                        else if (gamepad2.dpad_up)
                            currentLiftState = LiftState.UP2;
                        else if (gamepad2.dpad_right)
                            currentLiftState = LiftState.UP3;
                        break;
                    case UP1:
                        robot.lift.target=Ridicare.POS_2;
                        robot.pendulare.setPosition(robot.pendul_outtake);
                        robot.aligner.setPosition(robot.aligner_outake);
                        if (gamepad2.dpad_down){
                            currentLiftState = LiftState.DOWN1;
                            currentServoPos = ServoPos.IN_PROGRESS;
                        } else if (gamepad2.dpad_up)
                            currentLiftState = LiftState.UP2;
                        else if (gamepad2.dpad_right)
                            currentLiftState = LiftState.UP3;
                        break;
                    case UP2:
                        robot.lift.target=Ridicare.POS_3;//trb masurat cu encoderu
                        robot.pendulare.setPosition(robot.pendul_outtake);
                        robot.aligner.setPosition(robot.aligner_outake);
//                        robot.usa.setPosition(robot.usa_intake);
                        if (gamepad2.dpad_down){
                            currentLiftState = LiftState.DOWN1;
                            currentServoPos = ServoPos.IN_PROGRESS;
                        }
                        else if (gamepad2.dpad_left)
                            currentLiftState = LiftState.UP1;
                        else if (gamepad2.dpad_right)
                            currentLiftState = LiftState.UP3;
                        break;
                    case UP3:
                        robot.lift.target=Ridicare.POS_4;//trb masurat cu encoderu
                        robot.pendulare.setPosition(robot.pendul_outtake);
                        robot.aligner.setPosition(robot.aligner_outake);
//                        robot.usa.setPosition(robot.usa_intake);
                        if (gamepad2.dpad_down){
                            currentLiftState = LiftState.DOWN1;
                            currentServoPos = ServoPos.IN_PROGRESS;
                        }
                        else if(gamepad2.dpad_left)
                            currentLiftState = LiftState.UP1;
                        else if (gamepad2.dpad_up)
                            currentLiftState = LiftState.UP2;
                        break;
                    case DOWN1:
                        robot.lift.target = Ridicare.POS_1;
                        robot.aligner.setPosition(robot.aligner_intake);
                        robot.pendulare.setPosition(0.35);
                        if(robot.lift.getCurrentPosition()<50) {
                            currentLiftState = LiftState.DOWN2;
                            midPos = robot.pendulare.getPosition();
                        }
                        break;
                    case DOWN2:

                        switch (currentServoPos){
                            case IDLE:
                                //nothing
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
                    currentMode = Mode.AUTOMATIC_CONTROL;
                }
                break;
            case AUTOMATIC_CONTROL:
                if(gamepad1.x) {
//                    drive.breakFollowing();
                    currentMode = Mode.DRIVER_CONTROL;
                }
                if (!drive.isBusy()) {
                    currentMode = Mode.DRIVER_CONTROL;
                }
                break;
        }
        drive.update();
        robot.lift.update();
        telemetry.addData("Case Driver Control: ", currentMode);
        telemetry.addData("Case Sisteme: ", currentLiftState);
        telemetry.addData("Case Servo", currentServoPos);
        telemetry.addData("Pozitie Hang",robot.spanzurare.getCurrentPosition());
        telemetry.addData("Poitie Lift", robot.lift.getCurrentPosition());
        telemetry.addData("Poitie Pendul", robot.pendulare.getPosition());
        telemetry.addData("Poitie MidPos", midPos);
        telemetry.update();

    }


}
