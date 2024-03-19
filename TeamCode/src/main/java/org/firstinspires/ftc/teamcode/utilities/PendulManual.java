package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.sisteme.Aligner;
import org.firstinspires.ftc.teamcode.sisteme.Pendulare;

public class PendulManual {
    Pendulare pendulare;
    Aligner aligner;
    public double pozitiepend = 0.65;
    double pozitieal;



    public PendulManual(Pendulare pendulare) {
        this.pendulare = pendulare;
//        this.pozitiepend = pendulare.getPosition();
    }



    public void MiscarePendManuala(Gamepad gamepad) {
        if (gamepad.left_bumper) {
            pendulare.setPosition(pozitiepend);
            pozitiepend = pozitiepend + 0.005;
        }
        if (gamepad.right_bumper) {
            pendulare.setPosition(pozitiepend);
            pozitiepend = pozitiepend - 0.005;
        }


    }
}
