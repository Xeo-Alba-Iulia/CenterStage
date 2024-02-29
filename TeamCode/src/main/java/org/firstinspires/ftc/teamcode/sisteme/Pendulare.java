package org.firstinspires.ftc.teamcode.sisteme;

import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Pendulare {
    ServoImplEx pend1, pend2;

    public Pendulare(ServoImplEx pend1, ServoImplEx pend2) {
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
