package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.sisteme.Pendulare;
import org.firstinspires.ftc.teamcode.sisteme.Ridicare;

public class robothardware {
    private final OpMode myOpMode;
    //motoare sasiu
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;


    //motoare sisteme
//    public DcMotor intec;
    public DcMotor ridicare1;
    public DcMotor ridicare2;
//    public DcMotor spanzurare;
  //servomotoare
//    public ServoImplEx al1;
//    public ServoImplEx al2;
//    public Servo usa;
//    public Servo plane;
    public ServoImplEx pend1;
    public ServoImplEx pend2;

//pozitii aligner caseta
    public final double aligner_outake1 = 0.72;
    public final double aligner_outake2 = 0.77;
    public final double aligner_outake3 = 0.77;
    public final double aligner_intake = 0.5;
    //pozitii pendul
    public final double pendul_outtake1 = 0.6;
    public final double pendul_outtake2 = 0.61;
    public final double pendul_outtake3 = 0.61;
    public final double pendul_intake = 0.015 ;

//    public Spanzurare hanging;
    public Ridicare lift;
//    public Aligner aligner;
    public Pendulare pendulare;

    public robothardware(OpMode opmode) {myOpMode = opmode;}

    public void init(){
        //Sasiu

//        frontLeft = myOpMode.hardwareMap.dcMotor.get("MotorFrontLeft");
//        frontRight = myOpMode.hardwareMap.dcMotor.get("MotorFrontRight");
//        backLeft = myOpMode.hardwareMap.dcMotor.get("MotorBackLeft");
//        backRight = myOpMode.hardwareMap.dcMotor.get("MotorBackRight");
//        spanzurare = myOpMode.hardwareMap.dcMotor.get("Spanzurare");
//        usa = myOpMode.hardwareMap.servo.get("Claw");
//        intec = myOpMode.hardwareMap.dcMotor.get("Intec");
//        al1 = myOpMode.hardwareMap.get(ServoImplEx.class, "Aligner1");//caseta
//        al2 = myOpMode.hardwareMap.get(ServoImplEx.class, "Aligner2");
//        plane = myOpMode.hardwareMap.servo.get("Avion");
        ridicare1 = myOpMode.hardwareMap.dcMotor.get("Ridicare1");
        ridicare2 = myOpMode.hardwareMap.dcMotor.get("Ridicare2");
        pend1 = myOpMode.hardwareMap.get(ServoImplEx.class, "Pendul1");
        pend2 = myOpMode.hardwareMap.get(ServoImplEx.class,"Pendul2");
        pendulare = new Pendulare(pend1, pend2);
//        aligner = new Aligner(al1,al2);
        lift = new Ridicare(ridicare1,ridicare2);
//        hanging = new Spanzurare(spanzurare);
//        plane.setDirection(Servo.Direction.REVERSE);
//        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //pend1.getPwmRange()
        ridicare1.setDirection(DcMotorSimple.Direction.REVERSE);

//        al1.setDirection(Servo.Direction.REVERSE);
//        ridicare2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


//    public void movement(Gamepad gamepad1) {
//        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
//        double rx = gamepad1.right_stick_x;
//
//
////        double sin = Math.sin(joystick_angle(x,y)-Math.PI/2);
////        double cos = Math.cos(joystick_angle(x,y)-Math.PI/2);
//
//
//
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double frontLeftPower = (y + x + rx) / denominator;
//        double backLeftPower = (y - x + rx) / denominator;
//        double frontRightPower = (y - x - rx) / denominator;
//        double backRightPower = (y + x - rx) / denominator;
//
//
//        frontLeft.setPower(frontLeftPower);
//        backLeft.setPower(backLeftPower);
//        frontRight.setPower(frontRightPower);
//        backRight.setPower(backRightPower);
//
//    }
}