package org.firstinspires.ftc.teamcode.sisteme;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Spanzurare {
    DcMotor spanzurare;
    int nr = 0;

    public Spanzurare(DcMotor spanzurare){
        this.spanzurare = spanzurare;
    }
    public void goToPosHanging(Gamepad gamepad){
        spanzurare.setTargetPosition(0);
        spanzurare.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (gamepad.y){
            nr++;
        }
        if(nr==1){
            spanzurare.setTargetPosition(2000);
            spanzurare.setPower(1);
        } else if (nr==2) {
            spanzurare.setTargetPosition(0);
            spanzurare.setPower(1);
        }
    }
}
