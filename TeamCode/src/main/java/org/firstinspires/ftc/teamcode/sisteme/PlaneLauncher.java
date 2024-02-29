package org.firstinspires.ftc.teamcode.sisteme;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class PlaneLauncher {
    Servo plane;
    public final double avion_armat = 0;
    public final double avion_dezarmat = 0.4;

    public PlaneLauncher(Servo plane) {
        this.plane = plane;
    }

    public void PlaneLaunching(Gamepad gamepad) {
        if(gamepad.y)
            plane.setPosition(avion_armat);
    }

    public double getPosition() {

        return plane.getPosition();
    }
}
