package frc.robot.commands;

import frc.robot.subsystems.LimelightSubsystem;

public class LimeCommand extends BaseTelemetryCommand{
    public LimeCommand(){
        addRequirements(new LimelightSubsystem());
    }
}
