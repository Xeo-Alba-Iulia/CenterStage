package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.sisteme.Aligner;
import org.firstinspires.ftc.teamcode.sisteme.Pendulare;
import org.firstinspires.ftc.teamcode.sisteme.Ridicare;
import org.firstinspires.ftc.teamcode.sisteme.Spanzurare;

public class robothardware {
    private final OpMode myOpMode;
    //motoare sasiu
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    //motoare sisteme
    public DcMotor intec;
    public DcMotor ridicare1;
    public DcMotor ridicare2;
    public DcMotor spanzurare;
  //servomotoare
    public Servo al1;
    public Servo al2;
    public Servo usa;
    public Servo plane;
    public ServoImplEx pend1;
    public ServoImplEx pend2;

//pozitii aligner caseta
    public final double aligner_outake1 = 0.7;
    public final double aligner_outake2 = 0.7;
    public final double aligner_outake3 = 0.7;
    public final double aligner_intake = 0;
    //pozitii pendul
    public final double pendul_outtake1 = 0.8;
    public final double pendul_outtake2 = 0.8;
    public final double pendul_outtake3 = 0.8;

    public final double pendul_intake = 0;

    public Spanzurare hanging;
    public Ridicare lift;
    public Aligner aligner;
    public Pendulare pendulare;

    public robothardware(OpMode opmode) {myOpMode = opmode;}

    public void init(){
        //Sasiu

        frontLeft = myOpMode.hardwareMap.dcMotor.get("MotorFrontLeft");
        frontRight = myOpMode.hardwareMap.dcMotor.get("MotorFrontRight");
        backLeft = myOpMode.hardwareMap.dcMotor.get("MotorBackLeft");
        backRight = myOpMode.hardwareMap.dcMotor.get("MotorBackRight");
        spanzurare = myOpMode.hardwareMap.dcMotor.get("Spanzurare");
        usa = myOpMode.hardwareMap.servo.get("Claw");
        intec = myOpMode.hardwareMap.dcMotor.get("Intec");
        al1 = myOpMode.hardwareMap.servo.get("Aligner1");//caseta
        al2 = myOpMode.hardwareMap.servo.get("Aligner2");
        plane = myOpMode.hardwareMap.servo.get("Avion");
        ridicare1 = myOpMode.hardwareMap.dcMotor.get("Ridicare1");
        ridicare2 = myOpMode.hardwareMap.dcMotor.get("Ridicare2");
        pend1 = myOpMode.hardwareMap.get(ServoImplEx.class, "Pendul1");
        pend2 = myOpMode.hardwareMap.get(ServoImplEx.class,"Pendul2");
        pendulare = new Pendulare(pend1, pend2);
        aligner = new Aligner(al1,al2);
        lift = new Ridicare(ridicare1,ridicare2);
        hanging = new Spanzurare(spanzurare);
        plane.setDirection(Servo.Direction.REVERSE);
        pend1.setDirection(Servo.Direction.REVERSE);
        pend2.setDirection(Servo.Direction.REVERSE);

        al1.setDirection(Servo.Direction.REVERSE);



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
        double backLeftPower = -y + x - rx;
        double frontRightPower = -y + x + rx;
        double backRightPower = y + x - rx;

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