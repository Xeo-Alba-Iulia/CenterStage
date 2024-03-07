package org.firstinspires.ftc.teamcode.sisteme;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robothardware;

public class Spanzurare {
    DcMotor spanzurare;
    enum HangingState{
    UP,
    DOWN
    }
    HangingState currentHang = HangingState.UP;
    private OpMode myOpMode;
    robothardware robot = new robothardware(myOpMode);
    int nr = 0;


    public Spanzurare(DcMotor spanzurare, OpMode myOpMode){
        this.spanzurare = spanzurare;
        this.myOpMode = myOpMode;
    }
    public void goToPosHanging(Gamepad gamepad){

        switch (currentHang){
            case DOWN:
                if(gamepad.y) {
                    spanzurare.setTargetPosition(2100);
                    spanzurare.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    spanzurare.setPower(-1);
                    currentHang = HangingState.UP;
                }
            case UP:
                if(gamepad.x) {
                    spanzurare.setTargetPosition(0);
                    spanzurare.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    spanzurare.setPower(1);
                    currentHang = HangingState.DOWN;
                }
        }
    }
}
