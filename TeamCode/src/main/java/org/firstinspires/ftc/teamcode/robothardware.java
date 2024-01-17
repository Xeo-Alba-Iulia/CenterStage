package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.graphics.Camera;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.sisteme.VirtualFourBar;
import org.firstinspires.ftc.vision.VisionPortal;

public class robothardware {
    private final OpMode myOpMode;
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor intec;
    public DcMotor ridicare1;
    public DcMotor ridicare2;
    public Servo aligner;


    public Servo geara;
    public ServoImplEx vFB1;
    public ServoImplEx vFB2;


    public final double aligner_outake = 0.58;
    public final double aligner_intake = 0;
    public final double vfb_intake = 0;
    public final double vfb_outake=0.71;
    public double ringhiman = 0.71;


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
        aligner = myOpMode.hardwareMap.servo.get("Aligner");
        ridicare1 = myOpMode.hardwareMap.dcMotor.get("Ridicare1");
        ridicare2 = myOpMode.hardwareMap.dcMotor.get("Ridicare2");
        vFB1 = myOpMode.hardwareMap.get(ServoImplEx.class, "vfb1");
        vFB2 = myOpMode.hardwareMap.get(ServoImplEx.class,"vfb2");
        virtualFourBar = new VirtualFourBar(vFB1,vFB2);
        vFB1.setDirection(Servo.Direction.REVERSE);
        vFB2.setDirection(Servo.Direction.REVERSE);





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

        double frontLeftPower = -y - x - rx;
        double backLeftPower = y - x + rx;
        double frontRightPower =- y + x + rx;
        double backRightPower = -y - x + rx;

        if (gamepad1.left_bumper) {
            frontRightPower *= 0.2 ;
            frontLeftPower *= 0.2;
            backRightPower *= 0.2;
            backLeftPower *= 0.2;
        }

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }
}