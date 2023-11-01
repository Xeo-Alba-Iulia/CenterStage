package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class DrivetrainSubsystem extends SubsystemBase {
    private final MecanumDrive drivetrain;

    public DrivetrainSubsystem(MotorEx frontLeftMotor, MotorEx frontRightMotor,
                               MotorEx backLeftMotor, MotorEx backRightMotor) {
        drivetrain = new MecanumDrive(false, frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        register();

    }

    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        drivetrain.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed, true);
    }
}
