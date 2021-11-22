package frc.robot.commands;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.commands.BaseTelemetryCommand;
import edu.wpi.first.wpilibj.controller.*;
import java.util.function.DoubleSupplier;

public class BaseDriveCommand extends BaseTelemetryCommand{
    protected final DrivetrainSubsystem subsystem;
    protected final DoubleSupplier x, y, rot;
    public PIDController pidController;
    public BaseDriveCommand(DrivetrainSubsystem subsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot){
        this.subsystem = subsystem;
        this.x = x;
        this.y = y;
        this.rot = rot;
        
        addRequirements(subsystem);
    }

    public void setPID(double kP, double kI, double kD){
        pidController = new PIDController(kP, kI, kD);
    }

    @Override
    public void end(boolean interrupted){
        subsystem.drive(new ChassisSpeeds(0,0,0));
        super.end(interrupted);
    }
}
