package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "TeleOP", group = "A")
public class TeleOP extends OpMode {

    RobotHardware robot = new RobotHardware(this);
    SampleMecanumDrive drive;

@Override
public void init(){
    robot.init();
    drive = new SampleMecanumDrive(hardwareMap);
}
public void loop() {
    drive.update();
    robot.movement(gamepad1);
}
}