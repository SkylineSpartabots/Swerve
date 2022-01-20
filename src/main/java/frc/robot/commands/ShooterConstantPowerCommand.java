package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;

public class ShooterConstantPowerCommand extends CommandBase {
    private final ShooterSubsystem m_subsystem;
    //private final Intake m_intake = Intake.getInstance();
    private final double constantPowerPercent = 0.3;

    public ShooterConstantPowerCommand(){
        m_subsystem = ShooterSubsystem.getInstance();
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize(){
        m_subsystem.setMotorPowerPercent(constantPowerPercent);
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted){
    }
}
