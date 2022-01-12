package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SingleShootCommand extends CommandBase{
    private final ShooterSubsystem m_subsystem;
    //private final Intake m_intake = Intake.getInstance();

    private final Timer m_timer = new Timer();

    private final double shooterPower;
    private final double m_durationPowerOn;

    public double shooterPowerCalc(double distance){
        //shooterPower = calculations
    }

    public SingleShootCommand(double distance){
        m_subsystem = ShooterSubsystem.getInstance();
        addRequirements(m_subsystem);
        shooterPower = shooterPowerCalc(distance);


    }

    @Override
    public void initialize(){
        m_timer.reset();
        m_timer.start();
    }

    @Override
    private void execute(){
        /*
        if(getIsBallIntaked){
            m_subsystem.setMotorPower(shooterPower);
            }
        }

        m_intake needs to be a boolean obtained from the intake subsystem.
        wait about 1.5 seconds for two ball, for one ball 1 second should be enough.
        */
    }

    @Override
    private void end(boolean interrupted){
        m_timer.stop();
    }

    @Override
    private boolean isFinished(){
        return m_timer.hasElapsed(m_durationPowerOn);
    }

}
