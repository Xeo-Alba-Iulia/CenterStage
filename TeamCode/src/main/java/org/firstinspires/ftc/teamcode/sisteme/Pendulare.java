package org.firstinspires.ftc.teamcode.sisteme;

import com.qualcomm.robotcore.hardware.Servo;

public class Pendulare {
    Servo pend1, pend2;

    public Pendulare(Servo pend1, Servo pend2) {
        this.pend1 = pend1;
        this.pend2 = pend2;
    }

    public void setPosition(double position) {
        pend1.setPosition(position);
        pend2.setPosition(position);
    }

    public double getPosition() {

        return pend1.getPosition();
    }
}
