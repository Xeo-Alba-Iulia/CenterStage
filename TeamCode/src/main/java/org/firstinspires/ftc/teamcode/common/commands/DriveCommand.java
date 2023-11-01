package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier forward;
    private final DoubleSupplier turn;
    private final DoubleSupplier strafe;


    public DriveCommand(DrivetrainSubsystem drivetrain,
                        DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn) {
        this.drivetrain = drivetrain;
        this.forward =forward;
        this.strafe=strafe;
        this.turn =turn;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.drive(strafe.getAsDouble(), forward.getAsDouble(), turn.getAsDouble());
    }
}
