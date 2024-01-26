package org.firstinspires.ftc.teamcode.sisteme;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utilities.PIDController;

public class Spanzurare {
    DcMotor spanzurare;

    public PIDController controller;

    public static final int POS_HANGING = 1000;
    public static final int POS_JOS = 0;
    public int target;

    public Spanzurare(DcMotor spanzurare){
        this.spanzurare = spanzurare;

        target = 0;
        controller = new PIDController(0,0,0);
    }
    public void setPower(double power){
        spanzurare.setPower(power);
    }
    public double getPower(){
        return spanzurare.getPower();
    }
    public int getCurrentPosition(){
        return spanzurare.getCurrentPosition();
    }
    public void resetEndcoder(){
        spanzurare.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void update(){
        double power = controller.update(target,spanzurare.getCurrentPosition());
        spanzurare.setPower(power);
    }
}
