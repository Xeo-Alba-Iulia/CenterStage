package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.sisteme.Pendulare;

public class PendulManual {
    Pendulare pendulare;

    public PendulManual(Pendulare pendulare) {
        this.pendulare = pendulare;
    }

    public void MiscarePendManuala(Gamepad gamepad){
        if(gamepad.right_bumper){
            pendulare.setPosition(pendulare.getPosition()+0.05);
        }
        if(gamepad.right_bumper)
            pendulare.setPosition(pendulare.getPosition()-0.05);
    }


}
