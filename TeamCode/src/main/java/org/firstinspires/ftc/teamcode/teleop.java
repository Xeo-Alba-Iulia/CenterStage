package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.sisteme.Ridicare;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utilities.PoseStorage;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "TeleOP", group = "A")
public class TeleOP extends OpMode {

    RobotHardware robot = new RobotHardware(this);
    SampleMecanumDrive drive;
}
@Override
public void init(){
    robot.init();
    dirve = new SampleMecanumDrive(hardwareMap);
}
public void loop(){
    drive.update();
    robot.movement(gamepad1);
}