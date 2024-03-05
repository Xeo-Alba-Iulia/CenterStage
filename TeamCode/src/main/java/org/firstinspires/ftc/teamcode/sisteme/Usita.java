package org.firstinspires.ftc.teamcode.sisteme;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Usita {
    Servo usa;
    public double usa_outtake = 0.4;
    public double usa_intake = 0;

    public Usita(Servo usa) {
        this.usa = usa;
    }

    public void doorOpening(Gamepad gamepad) {
        if(gamepad.right_bumper)
            usa.setPosition(usa_outtake);
        else
            usa.setPosition(usa_intake);

    }

    public double getPosition() {

        return usa.getPosition();
    }
}
