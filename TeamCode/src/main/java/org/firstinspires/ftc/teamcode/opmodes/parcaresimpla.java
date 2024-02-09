package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robothardware;


@Autonomous(name = "parcare simpla", group = "A", preselectTeleOp = "prototype")
public class parcaresimpla extends LinearOpMode {
    ElapsedTime time = new ElapsedTime();
    robothardware robot = new robothardware(this);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        if(isStopRequested()) {
            return;
        }


        time.reset();
        while(time.seconds() < 0.6) {
            robot.frontRight.setPower(0.5);
            robot.frontLeft.setPower(-0.5);
            robot.backLeft.setPower(-0.5);
            robot.backRight.setPower(0.5);
        }
        }

    }