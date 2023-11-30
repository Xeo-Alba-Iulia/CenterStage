package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.sisteme.VirtualFourBar;
public class robothardware {
    private final OpMode myOpMode;
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor intec;
    public DcMotor ridicare1;
    public DcMotor ridicare2;



    public Servo geara;
    public ServoImplEx vfb1;
    public ServoImplEx vfb2;


    public final double vfb_intake = 0.1;
    public final double vfb_outake=0.8;

    public VirtualFourBar virtualFourBar;

    public robothardware(OpMode opmode) {myOpMode = opmode;}

    public void init(){
        //Sasiu

        frontLeft = myOpMode.hardwareMap.dcMotor.get("MotorFrontLeft");
        frontRight = myOpMode.hardwareMap.dcMotor.get("MotorFrontRight");
        backLeft = myOpMode.hardwareMap.dcMotor.get("MotorBackLeft");
        backRight = myOpMode.hardwareMap.dcMotor.get("MotorBackRight");
        geara = myOpMode.hardwareMap.servo.get("Claw");
        intec = myOpMode.hardwareMap.dcMotor.get("Intec");
        ridicare1 = myOpMode.hardwareMap.dcMotor.get("Ridicare1");
        ridicare2 = myOpMode.hardwareMap.dcMotor.get("Ridicare2");
        vfb1 = myOpMode.hardwareMap.get(ServoImplEx.class, "vfb1");
        vfb2 = myOpMode.hardwareMap.get(ServoImplEx.class,"vfb2");
        virtualFourBar = new VirtualFourBar(vfb1,vfb2);
    }
    public void movement(Gamepad gamepad1) {
        double y = -gamepad1.left_stick_y;
        double x;
        if (gamepad1.left_stick_x < 0.25 && gamepad1.left_stick_x > -0.25) {
            x = 0;
        } else {
            x = gamepad1.left_stick_x;
        }
        double rx = gamepad1.right_stick_x;

        double frontLeftPower = y - x + rx;
        double backLeftPower = y + x + rx;
        double frontRightPower = -y - x + rx;
        double backRightPower = -y + x + rx;

        if (gamepad1.left_trigger!=0) {
            frontRightPower *= 0.4;
            frontLeftPower *= 0.4;
            backRightPower *= 0.4;
            backLeftPower *= 0.4;
        } else if (gamepad1.right_trigger!=0) {
            frontRightPower *= 0.6;
            frontLeftPower *= 0.6;
            backRightPower *= 0.6;
            backLeftPower *= 0.6;

        }

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }
}