package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.robothardware;
import org.firstinspires.ftc.teamcode.utilities.PoseStorage;


@TeleOp(name = "TeleOP", group = "A")

public class Teleop extends OpMode {

    robothardware robot = new robothardware(this);



    private static int state_lift_pos = 100;
    private double state_caseta_align = robot.aligner_intake;
    private double state_pendul_pos = robot.pendul_intake;
    boolean right_bumper_pressed = false;
    private org.firstinspires.ftc.robotcore.external.Telemetry Telemetry;

    TwoWheelTrackingLocalizer myLocalizer;
    double x,y,rx;

    double sin;
    double cos;
    @Override
    public void init() {
        robot.init();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        myLocalizer = new TwoWheelTrackingLocalizer(hardwareMap, drive);
//        robot.usa.setPosition(0.0);
//        robot.al1.setPosition(0.0);
//        robot.al2.setPosition(0.0);
//        robot.plane.setPosition(0);
//        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        robot.ridicare2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myLocalizer.setPoseEstimate(PoseStorage.currentPose);




    }

//    private void storageOuttake() {
//
//        if(gamepad2.dpad_down){
//            robot.lift.target = 0;
//            robot.pendulare.setPosition(robot.pendul_intake);
//            robot.aligner.setPosition(robot.aligner_intake);
//            robot.usa.setPosition(0.565);
//
//
//        }
//        if (gamepad2.dpad_left){
//            robot.lift.target = Ridicare.POS_1;
//            robot.pendulare.setPosition(robot.pendul_outtake2);
//            robot.aligner.setPosition(robot.aligner_outake2);
//            robot.usa.setPosition(0.61);
//        }
//        else if (gamepad2.dpad_up){
//            robot.lift.target = Ridicare.POS_2;
//            robot.pendulare.setPosition(0.8);
//            robot.aligner.setPosition(robot.aligner_outake2);
//            robot.usa.setPosition(0.61);
//        }
//        else if (gamepad2.dpad_right){
//            robot.lift.target = Ridicare.POS_3;
//            robot.pendulare.setPosition(robot.pendul_outtake3);
//            robot.aligner.setPosition(robot.aligner_outake3);
//            robot.usa.setPosition(0.61 );
//        }
//
//    }
//
//    private void intake(){
//            if (gamepad2.a)
//                robot.intec.setPower(1);
//            else if (gamepad2.b)
//                robot.intec.setPower(-1);
//            else
//                robot.intec.setPower(0);
//
//    }
//    private void ridicare(){
//        robot.ridicare1.setPower(gamepad2.right_trigger);
//        robot.ridicare2.setPower(gamepad2.right_trigger);
//
//
//        robot.ridicare1.setPower(-gamepad2.left_trigger);
//        robot.ridicare2.setPower(-gamepad2.left_trigger);
//    }
//    private void spanzurare(){
//        if (gamepad1.x)
//            robot.hanging.target = Spanzurare.POS_HANGING;
//        else if (gamepad1.b)
//            robot.hanging.target = Spanzurare.POS_JOS;
//    }
//

    public void loop() {

        robot.movement(gamepad1);
//        spanzurare();
//        storageOuttake();
//        intake();
//        spanzurare();
//        ridicare();
//        robot.spanzurare.setPower(gamepad1.right_stick_y);



//        if(gamepad1.right_bumper) {
//            robot.usa.setPosition(0);
//        }
//
//        if(gamepad1.x){
//            robot.plane.setPosition(0.4);
//        }
        myLocalizer.update();
        Pose2d myPose = myLocalizer.getPoseEstimate();

        telemetry.addData("x", myPose.getX());
        telemetry.addData("y", myPose.getY());
        telemetry.addData("heading", myPose.getHeading());
//        telemetry.addData("Pendulare", robot.pendulare.getPosition());
//        telemetry.addData("Aligner", robot.aligner.getPosition());
//        telemetry.addData("Pozitie Ridicare", robot.ridicare2.getCurrentPosition());
        x  = -gamepad1.left_stick_y;
        y  = gamepad1.left_stick_x;
        rx = gamepad1.right_stick_y;
        sin = Math.sin(robot.joystick_angle(x,y)-Math.PI/2);
        cos = Math.cos(robot.joystick_angle(x,y)-Math.PI/2);


        telemetry.addData("Speed",robot.joystick_speed(x,y));
        telemetry.addData("Turn", robot.joystick_angle(x,y));
        telemetry.addData("X",sin);
        telemetry.addData("Y", cos);
        telemetry.addData("Rx", rx);
        telemetry.update();
    }


}
