package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;
import org.firstinspires.ftc.teamcode.robothardware;


@Autonomous(name = "parcare simpla", group = "A", preselectTeleOp = "prototype")
public class parcaresimpla extends LinearOpMode {
    ElapsedTime time = new ElapsedTime();
    robothardware robot = new robothardware(this);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        waitForStart();
        if(isStopRequested()) {
            return;
        }
        time.reset();
        while(time.seconds() < 0.8) {
            robot.frontRight.setPower(-0.5);
            robot.frontLeft.setPower(-0.5);
            robot.backLeft.setPower(0.5);
            robot.backRight.setPower(-0.5);
        }
    }

}