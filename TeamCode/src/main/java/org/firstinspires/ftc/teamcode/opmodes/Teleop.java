package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.common.subsystems.DrivetrainSubsystem;

@TeleOp
public class Teleop extends CommandOpMode {

    @Override
    public void initialize() {

        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx toolsop = new GamepadEx(gamepad2);


        DrivetrainSubsystem drive = new DrivetrainSubsystem(
                new MotorEx(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_435),
                new MotorEx(hardwareMap, "frontRight", Motor.GoBILDA.RPM_435),
                new MotorEx(hardwareMap, "backLeft", Motor.GoBILDA.RPM_435),
                new MotorEx(hardwareMap, "backRi`ght", Motor.GoBILDA.RPM_435)
        );
        DriveCommand driveCommand = new DriveCommand(drive,
                driverOp::getLeftY, driverOp::getLeftX, driverOp::getRightX);
        drive.setDefaultCommand(driveCommand);
    }

}
