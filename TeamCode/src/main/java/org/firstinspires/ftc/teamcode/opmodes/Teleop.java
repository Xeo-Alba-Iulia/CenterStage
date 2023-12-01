package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOP", group = "A")
public class Teleop extends OpMode {

    robothardware robot = new robothardware(this);

    boolean right_bumper_pressed = false;


    @Override
    public void init() {
        robot.init();
        robot.geara.setPosition(0);
        robot.vfb1.setPosition(0);
        robot.vfb2.setPosition(0);


    }

    public void loop() {

        robot.movement(gamepad1);
//
        if (gamepad1.a)
            robot.intec.setPower(0.8);

        if (gamepad1.b)
            robot.intec.setPower(-0.8);
        if (gamepad1.y)
            robot.intec.setPower(0);


        robot.ridicare1.setPower(gamepad1.right_trigger);
        robot.ridicare2.setPower(-gamepad1.right_trigger);

        robot.ridicare1.setPower(-gamepad1.left_trigger);
        robot.ridicare2.setPower(gamepad1.left_trigger);

        //gheara
        if(gamepad1.right_bumper) {
            robot.geara.setPosition(1);
        } else {
            robot.geara.setPosition(0);
        }

        //ridicare
        if (gamepad1.right_trigger > 0) {
            robot.ridicare1.setPower(0.3);
            robot.ridicare1.setPower(-0.3);
        } else {
            robot.ridicare1.setPower(0);
        }
        if (gamepad1.left_trigger > 0) {
            robot.ridicare1.setPower(-0.3);
            robot.ridicare1.setPower(0.3);
        } else {
            robot.ridicare2.setPower(0);
        }


        //vfb cica
        if (gamepad1.left_bumper) {
            robot.virtualFourBar.setPosition(robot.vfb_intake);
        }

        ///ii ora 1 si imi bag imbusu-n el cod #graciuosly
        telemetry.addData("geara pos", robot.geara.getPosition());
        telemetry.addData("bool", right_bumper_pressed);
        telemetry.update();
    }
}
