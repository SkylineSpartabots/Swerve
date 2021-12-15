package frc.robot.commands;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.commands.BaseTelemetryCommand;
import edu.wpi.first.wpilibj.controller.*;
import java.util.function.DoubleSupplier;

public class DriveCommand extends BaseTelemetryCommand{
    protected final DrivetrainSubsystem subsystem;
    protected final DoubleSupplier xPower, yPower, rotPower;
    protected double xTarget = 0, yTarget, angleTarget;

    public DriveCommand(DrivetrainSubsystem subsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot){
        this.subsystem = subsystem;
        this.xPower = x;
        this.yPower = y;
        this.rotPower = rot;
        
        addRequirements(subsystem);
    }

    public void setTarget(double x, double y, double ang){
        this.xTarget = x;
        this.yTarget = y;
        this.angleTarget = ang;
    }
    @Override
    public void execute() {
        super.execute();
        subsystem.drive( ChassisSpeeds.fromFieldRelativeSpeeds(
                    xPower.getAsDouble(),
                    yPower.getAsDouble(),
                    rotPower.getAsDouble(),
                    subsystem.getGyroscopeRotation()
                )
        );
    }

    @Override
    public boolean isFinished(){
        if(xTarget != 0){
            //Logic for finishing
            double currentX = subsystem.getPose().getTranslation().getX();
            if(Math.abs(currentX) >= xTarget - 0.1 && Math.abs(currentX) <= xTarget + 0.1){
                return true;
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        subsystem.drive(new ChassisSpeeds(0,0,0));
        super.end(interrupted);
    }
}
