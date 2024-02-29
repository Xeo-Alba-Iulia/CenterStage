package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robothardware;

@TeleOp(name = "Test Sisteme", group = "A")

public class ManualSisteme extends OpMode {
    robothardware robot = new robothardware(this);

    @Override
    public void init() {
        robot.init();
        robot.spanzurare.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pendulare.setPosition(robot.pendul_intake);
        robot.aligner.setPosition(robot.aligner_intake);
        robot.usa.setPosition(robot.usa_intake);
//        robot.spanzurare.setTargetPosition(2200);
//        robot.spanzurare.setMode(DcMotor.RunMode.RUN_TO_POSITION);



//        robot.plane.setPosition(robot.avion_armat);
    }

    private void intake(){
        if (gamepad1.a)
            robot.intec.setPower(1);
        else if (gamepad1.b)
            robot.intec.setPower(-1);
        else
            robot.intec.setPower(0);

    }
    public void ridicare(){
        robot.ridicare1.setPower(-gamepad1.left_trigger);
        robot.ridicare2.setPower(-gamepad1.left_trigger);

        robot.ridicare1.setPower(gamepad1.right_trigger);
        robot.ridicare2.setPower(gamepad1.right_trigger);
    }
    public void aligner(){
        if (gamepad1.dpad_left)
            robot.aligner.setPosition(robot.aligner_intake);
        if(gamepad1.dpad_right)
            robot.aligner.setPosition(robot.aligner_outake);
    }
    public void pendulare(){
        if (gamepad1.dpad_down)
            robot.pendulare.setPosition(robot.pendul_intake);
        if (gamepad1.dpad_up)
            robot.pendulare.setPosition(robot.pendul_outtake);
    }
    public void usa(){
        if(gamepad1.right_bumper)
            robot.usa.setPosition(robot.usa_outtake);
        else
            robot.usa.setPosition(robot.usa_intake);
    }
    public void hanging(){
        robot.spanzurare.setPower(gamepad1.right_stick_y);
    }
//    public void aveon(){
//        robot.plane.setPosition(robot.avion_dezarmat);
//    }


    @Override
    public void loop() {
        robot.movement(gamepad1);
        ridicare();
        aligner();
        pendulare();
        usa();
        hanging();
        robot.intake.runIntake(gamepad1);
        robot.hanging.goToPosHanging(gamepad1);
//        switch (currentServoPos){
//            case IDLE:
//                midPos = robot.pendulare.getPosition();
//                break;
//            case IN_PROGRESS:
//                robot.pendulare.setPosition(ServoSmoothing.servoSmoothing(midPos, robot.pendul_intake));
//                if(robot.pendulare.getPosition()>robot.pendul_intake +0.005) {
//                    robot.pendulare.setPosition(robot.pendul_intake);
//                    currentServoPos = Teleop.ServoPos.IDLE;
//                }
//                else {
//                    midPos = robot.pendulare.getPosition();
//                }
//                break;
//        }
//        if(gamepad1.y)
//            robot.spanzurare.setPower(1);
//        if(gamepad1.left_bumper) {
//            robot.spanzurare.setTargetPosition(0);
//            robot.spanzurare.setPower(1);
//        }
//        aveon();
        telemetry.addData("Pozitie Ridicare", robot.ridicare1.getCurrentPosition());
        telemetry.addData("Pozitie Hanging", robot.spanzurare.getCurrentPosition());
        telemetry.update();
//        telemetry.addData("Curr Pos Hanging", robot.spanzurare.getTargetPosition());
    }


}
