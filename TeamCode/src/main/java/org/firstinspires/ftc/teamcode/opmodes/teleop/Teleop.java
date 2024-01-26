package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robothardware;
import org.firstinspires.ftc.teamcode.sisteme.Ridicare;
import org.firstinspires.ftc.teamcode.sisteme.Spanzurare;


@TeleOp(name = "TeleOP", group = "A")

public class Teleop extends OpMode {

    robothardware robot = new robothardware(this);

    private static int state_lift_pos = 100;
    private double state_caseta_align = robot.aligner_intake;
    private double state_pendul_pos = robot.pendul_intake;
    boolean right_bumper_pressed = false;
    private org.firstinspires.ftc.robotcore.external.Telemetry Telemetry;


    @Override
    public void init() {
        robot.init();
//        robot.geara.setPosition(0.0);
        robot.pend1.setPosition(0.08);
        robot.pend2.setPosition(0.08);
        robot.al1.setPosition(0.0);
        robot.al2.setPosition(0.0);
        robot.plane.setPosition(0);
//        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));




    }

    private void storageOuttake() {

        if(gamepad2.dpad_down){
            robot.lift.target = 0;
            robot.pendulare.setPosition(robot.pendul_intake);
            robot.aligner.setPosition(robot.aligner_intake);


        }
        else if (gamepad2.dpad_left){
            robot.lift.target = Ridicare.POS_1;
            robot.pendulare.setPosition(robot.pendul_outtake1);
            robot.aligner.setPosition(robot.aligner_outake1);
        }
        else if (gamepad2.dpad_up){
            robot.lift.target = Ridicare.POS_2;
            robot.pendulare.setPosition(robot.pendul_outtake2);
            robot.aligner.setPosition(robot.aligner_outake2);
        }
        else if (gamepad2.dpad_right){
            robot.lift.target = Ridicare.POS_3;
            robot.pendulare.setPosition(robot.pendul_outtake3);
            robot.aligner.setPosition(robot.aligner_outake3);
        }
        robot.lift.update();
    }

    private void intake(){
            if (gamepad2.a)
                robot.intec.setPower(0.9);

            if (gamepad2.b)
                robot.intec.setPower(-0.9);
            if (gamepad2.y)
                robot.intec.setPower(0);
        }
    private void ridicare(){
        robot.ridicare1.setPower(gamepad2.right_trigger);
        robot.ridicare2.setPower(gamepad2.right_trigger);

        robot.ridicare1.setPower(-gamepad2.left_trigger);
        robot.ridicare2.setPower(-gamepad2.left_trigger);
    }
    private void spanzurare(){
        if (gamepad1.x)
            robot.hanging.target = Spanzurare.POS_HANGING;
        else if (gamepad1.b)
            robot.hanging.target = Spanzurare.POS_JOS;
    }

    public void loop() {

        robot.movement(gamepad1);
        spanzurare();
        storageOuttake();
        intake();
        spanzurare();
        ridicare();





        if(gamepad1.right_bumper) {
            robot.usa.setPosition(0);
        }
        else{
            robot.usa.setPosition(0.34);
        }
        if(gamepad1.a){
            robot.plane.setPosition(0.5);
        }


        if(gamepad2.dpad_up){
           robot.pend1.setPosition(0.65);
           robot.pend2.setPosition(0.65);

        }
        if(gamepad2.dpad_down){
            robot.pend1.setPosition(0.08);
            robot.pend2.setPosition(0.08);

        }

        if (gamepad2.left_bumper) {
            robot.al1.setPosition(0);
            robot.al2.setPosition(0);
        }
        if (gamepad2.right_bumper){
            robot.al1.setPosition(0.65);
            robot.al2.setPosition(0.65);
        }








        ///ii ora 1 si imi bag imbusu-n el cod #graciuosly
//        telemetry.addData("vfb pos", robot.virtualFourBar.getPosition());
//        telemetry.addData("geara pos", robot.intec.getCurrentPosition());
//        telemetry.addData("bool", right_bumper_pressed);
//        telemetry.addData("pos aligner",robot.aligner.getPosition());
//        telemetry.update();
    }


}
