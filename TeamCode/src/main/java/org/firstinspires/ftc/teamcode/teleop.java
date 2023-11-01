package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "TeleOP", group = "A")
public class TeleOP extends OpMode {

    RobotHardware robot = new RobotHardware(this);


@Override
public void init(){
    robot.init();

}
public void loop() {
    robot.movement(gamepad1);
}
}