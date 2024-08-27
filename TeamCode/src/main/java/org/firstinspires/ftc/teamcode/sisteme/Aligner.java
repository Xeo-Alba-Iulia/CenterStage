package org.firstinspires.ftc.teamcode.sisteme;

import com.qualcomm.robotcore.hardware.Servo;

public class Aligner {
    Servo al1, al2;
    public Aligner(Servo al1, Servo al2) {
        this.al1 = al1;
        this.al2 = al2;
    }

    public void setPosition(double position) {
        al1.setPosition(-position);
        al2.setPosition(-position);
    }

    public double getPosition() {

        return al1.getPosition();
    }
}
