package frc.robot.commands;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends BaseDriveCommand{
    public DefaultDriveCommand(DrivetrainSubsystem subsystem, double x, double y, double rot){
        super(subsystem, x, y, rot);
    }

    @Override
    public void execute(){
        super.execute();
        subsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            x, y, rot, subsystem.getGyroscopeRotation()));
    }
}
