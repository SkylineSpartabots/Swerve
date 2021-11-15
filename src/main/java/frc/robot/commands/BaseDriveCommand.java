package frc.robot.commands;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.commands.BaseTelemetryCommand;

public class BaseDriveCommand extends BaseTelemetryCommand{
    public final DrivetrainSubsystem subsystem;
    public final double x, y, rot;
    public BaseDriveCommand(DrivetrainSubsystem subsystem, double x, double y, double rot){
        this.subsystem = subsystem;
        this.x = x;
        this.y = y;
        this.rot = rot;
        
        addRequirements(subsystem);
    }

    @Override
    public void end(boolean interrupted){
        subsystem.drive(new ChassisSpeeds(0,0,0));
        super.end(interrupted);
    }
}
