package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.sisteme.Aligner;
import org.firstinspires.ftc.teamcode.sisteme.Intake;
import org.firstinspires.ftc.teamcode.sisteme.Pendulare;
import org.firstinspires.ftc.teamcode.sisteme.PlaneLauncher;
import org.firstinspires.ftc.teamcode.sisteme.Ridicare;
import org.firstinspires.ftc.teamcode.sisteme.Spanzurare;
import org.firstinspires.ftc.teamcode.sisteme.Usita;
import org.firstinspires.ftc.teamcode.utilities.PendulManual;

public class robothardware {
    private final OpMode myOpMode;
    //motoare sasiu
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;


    //motoare sisteme
    public DcMotor intec;
    public DcMotor ridicare1;
    public DcMotor ridicare2;
    public DcMotor spanzurare;
  //servomotoare
    public ServoImplEx al1;
    public ServoImplEx al2;
    public Servo usa;
    public Servo plane;
    public ServoImplEx pend1;
    public ServoImplEx pend2;

    //pozitii aligner caseta
    public final double aligner_outake = 0.75;
    public final double aligner_intake = 0;

    //pozitii pendul
    public final double pendul_outtake = 0.6;
    public final double pendul_intake = 0.025;

    //Obiecte
    public Spanzurare hanging;
    public Ridicare lift;
    public Aligner aligner;
    public Pendulare pendulare;
    public Intake intake;
    public Usita door;
    public PlaneLauncher avion;
    public PendulManual pendulManual;

    public robothardware(OpMode opmode) {myOpMode = opmode;}

    public void init(){
        //Sasiu

        frontLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "MotorFrontLeft");
        frontRight = myOpMode.hardwareMap.get(DcMotorEx.class,"MotorFrontRight");
        backLeft = myOpMode.hardwareMap.get(DcMotorEx.class,"MotorBackLeft");
        backRight = myOpMode.hardwareMap.get(DcMotorEx.class,"MotorBackRight");

       //Motoare sasiu
        spanzurare = myOpMode.hardwareMap.dcMotor.get("Spanzurare");
        ridicare1 = myOpMode.hardwareMap.dcMotor.get("Ridicare1");
        ridicare2 = myOpMode.hardwareMap.dcMotor.get("Ridicare2");
        intec = myOpMode.hardwareMap.dcMotor.get("Intec");

        //Servouri sisteme
        usa = myOpMode.hardwareMap.servo.get("Usa");
        al1 = myOpMode.hardwareMap.get(ServoImplEx.class, "Aligner1");//caseta
        al2 = myOpMode.hardwareMap.get(ServoImplEx.class, "Aligner2");
//        plane = myOpMode.hardwareMap.servo.get("Avion");
        pend1 = myOpMode.hardwareMap.get(ServoImplEx.class, "Pendul1");
        pend2 = myOpMode.hardwareMap.get(ServoImplEx.class,"Pendul2");

        //Obiecte sisteme
        pendulare = new Pendulare(pend1, pend2);
        aligner = new Aligner(al1,al2);
        lift = new Ridicare(ridicare1,ridicare2);
        hanging = new Spanzurare(spanzurare,myOpMode);
        door = new Usita(usa);
        intake = new Intake(intec);
        avion = new PlaneLauncher(plane);
        pendulManual = new PendulManual(pendulare);

        //inversarea directiilor
//        plane.setDirection(Servo.Direction.REVERSE);
        usa.setDirection(Servo.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        ridicare2.setDirection(DcMotorSimple.Direction.REVERSE);

        //Encodere motoare
        ridicare1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        spanzurare.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        spanzurare.setTargetPosition(0);
//        spanzurare.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }


    public void movement(Gamepad gamepad1) {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;


        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

    }
}