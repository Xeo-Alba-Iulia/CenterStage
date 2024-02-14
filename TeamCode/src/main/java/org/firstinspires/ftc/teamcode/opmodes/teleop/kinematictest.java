package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robothardware;
@TeleOp(name = "TestKinematic", group = "A")
public class  kinematictest extends OpMode {

    robothardware robot = new robothardware(this);

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    double x,y;
    double rx;



    @Override
    public void init() {

        frontLeft = hardwareMap.dcMotor.get("MotorFrontLeft");
        frontRight = hardwareMap.dcMotor.get("MotorFrontRight");
        backLeft = hardwareMap.dcMotor.get("MotorBackLeft");
        backRight = hardwareMap.dcMotor.get("MotorBackRight");

    }

    @Override
    public void loop() {

    robot.movement(gamepad1);
        x  = -gamepad1.left_stick_y;
        y  = gamepad1.left_stick_x;
        rx = gamepad1.right_stick_x;
        telemetry.addData("Speed",robot.joystick_speed(x,y));
        telemetry.addData("Turn", robot.joystick_angle(x,y));
    telemetry.update();

    }
}
