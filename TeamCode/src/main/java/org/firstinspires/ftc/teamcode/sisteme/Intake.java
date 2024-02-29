package org.firstinspires.ftc.teamcode.sisteme;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Intake {
    DcMotor intec;

    public Intake (DcMotor intec) {
        this.intec = intec;
    }

    public void runIntake(Gamepad gamepad) {
        if (gamepad.a)
            intec.setPower(1);
        else if (gamepad.b)
            intec.setPower(-1);
        else
            intec.setPower(0);

    }
}
