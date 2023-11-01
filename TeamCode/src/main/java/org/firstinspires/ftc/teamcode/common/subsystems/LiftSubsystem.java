package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import java.util.Arrays;
import java.util.List;

public class LiftSubsystem extends SubsystemBase {
    List<MotorEx> liftMotors;

    public LiftSubsystem(MotorEx... liftMotors) {
        this.liftMotors.addAll(Arrays.asList(liftMotors));
    }

    public static double kp = 0, ki = 0 , kd = 0, kf = 0;

    PIDFController pidf = new PIDFController(kp, ki, kd, kf);

    /**
     * Set the lift's pidf reference
     * @param target Target in cm
     */
    public void goTo(double target) {
        pidf.setSetPoint(toTicks(target));
    }

    /**
     * Update the PIDF loop
     */
    public void update() {
        double state = liftMotors.get(1).getCurrentPosition();
        double input = pidf.calculate(state);
        for (MotorEx liftMotor:liftMotors) {
            liftMotor.set(input);
        }
    }

    private double toTicks(double cm) {
        double ENCODER_CPR = 8192;
        // cm
        double PULLEY_RADIUS = 0.30;
        double ticksPerCm = ENCODER_CPR / 2 * Math.PI * PULLEY_RADIUS;
        return cm * ticksPerCm;
    }

    private double getVerticalDistance(double distance) {
        double slidesAngleToHorizontalAxis =  30; // degrees
        return distance * Math.sin(Math.toRadians(slidesAngleToHorizontalAxis));
    }
}
