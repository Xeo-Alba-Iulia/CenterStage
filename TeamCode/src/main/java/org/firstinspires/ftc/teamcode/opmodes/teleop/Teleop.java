package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robothardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;



@TeleOp(name = "TeleOP", group = "A")
public class Teleop extends OpMode {

    robothardware robot = new robothardware(this);

    boolean right_bumper_pressed = false;
    private org.firstinspires.ftc.robotcore.external.Telemetry Telemetry;


    @Override
    public void init() {
//        robot.init();
//        robot.geara.setPosition(0);
//        robot.vFB1.setPosition(0);
//        robot.vFB2.setPosition(0);
//        robot.aligner.setPosition(0);

        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));




    }
    private void miscaremanuala() {
        if (gamepad2.dpad_left) {
            robot.ringhiman += 0.005;
            robot.vFB1.setPosition(robot.ringhiman);
        } else if (gamepad2.dpad_right) {
            robot.ringhiman -= 0.005;
            robot.virtualFourBar.setPosition(robot.ringhiman);
        }
    }
    private void intake(){
            if (gamepad2.a)
                robot.intec.setPower(0.85);

            if (gamepad2.b)
                robot.intec.setPower(-0.85);
            if (gamepad2.y)
                robot.intec.setPower(0);
        }
    private void ridicare(){
        robot.ridicare1.setPower(gamepad2.right_trigger);
        robot.ridicare2.setPower(gamepad2.right_trigger);

        robot.ridicare1.setPower(-gamepad2.left_trigger);
        robot.ridicare2.setPower(-gamepad2.left_trigger);
    }

    public void loop() {

        robot.movement(gamepad1);
        miscaremanuala();
        intake();
        ridicare();





        if(gamepad1.right_bumper) {
            robot.geara.setPosition(0.8);
        } else {
            robot.geara.setPosition(0);
        }

        if(gamepad2.dpad_up){
            robot.virtualFourBar.setPosition(robot.vfb_outake);
            robot.aligner.setPosition(robot.aligner_outake);
        }
        if(gamepad2.dpad_down){
            robot.virtualFourBar.setPosition(robot.vfb_intake);
            robot.aligner.setPosition(robot.aligner_intake);
        }







        ///ii ora 1 si imi bag imbusu-n el cod #graciuosly
        telemetry.addData("vfb pos", robot.virtualFourBar.getPosition());
        telemetry.addData("geara pos", robot.intec.getCurrentPosition());
        telemetry.addData("bool", right_bumper_pressed);
        telemetry.addData("pos aligner",robot.aligner.getPosition());
        telemetry.update();
    }
}
