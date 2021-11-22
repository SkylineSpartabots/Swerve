package frc.robot.commands;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends BaseDriveCommand{

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        super(drivetrainSubsystem, translationXSupplier, translationYSupplier, rotationSupplier);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        super.execute();
        subsystem.drive( ChassisSpeeds.fromFieldRelativeSpeeds(
                    x.getAsDouble(),
                    y.getAsDouble(),
                    rot.getAsDouble(),
                    subsystem.getGyroscopeRotation()
                )
        );
    }
}
