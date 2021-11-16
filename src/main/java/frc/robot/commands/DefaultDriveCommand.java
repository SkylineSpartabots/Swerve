package frc.robot.commands;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends CommandBase{
    public final DrivetrainSubsystem subsystem;
    public final double x, y, rot;
    public DefaultDriveCommand(DrivetrainSubsystem subsystem, double x, double y, double rot){
        //super(subsystem, x, y, rot);
        this.subsystem = subsystem;
        this.x =x;
        this.y=y;
        this.rot=rot;

        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        //super.execute();
        subsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            x, y, rot, subsystem.getGyroscopeRotation()));
    }

    @Override
    public void end(boolean interrupted){
        subsystem.drive(new ChassisSpeeds(0,0,0));
        //super.end(interrupted);
    }
}
