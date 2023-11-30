package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOP", group = "A")
public class Teleop extends OpMode {

    robothardware robot = new robothardware(this);


    @Override
    public void init(){
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
        if(gamepad1.y)
            robot.intec.setPower(0);

        robot.ridicare1.setPower(gamepad2.right_trigger);
        robot.ridicare2.setPower(-gamepad2.right_trigger);

        robot.ridicare1.setPower(-gamepad2.left_trigger);
        robot.ridicare2.setPower(gamepad2.left_trigger);

        if(gamepad2.b)
            robot.geara.setPosition(0.4);
        if(gamepad2.a)
            robot.geara.setPosition(0.2);
        if(gamepad1.left_bumper) {
            robot.vfb1.setPosition(-1);
            robot.vfb2.setPosition(1);
        }
        if(gamepad1.right_bumper) {
            robot.vfb1.setPosition(0);
            robot.vfb2.setPosition(0);
        }
    }



    }
