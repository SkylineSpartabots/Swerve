package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;

public class indexerPushCommand extends CommandBase{
    private final IndexerSubsystem m_subsystem;

    private final Timer m_timer = new Timer();

    private final double m_durationPush = 0.25;

    public indexerPushCommand(){
        m_subsystem = IndexerSubsystem.getInstance();
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize(){
        m_timer.reset();
        m_timer.start();
        m_subsystem.setIndexerPowerPercent(Constants.IndexerConstants.indexerSpeedPercent);
    }

    @Override
    public void execute(){
        //implement some sort of encoder in Indexer to make sure it doesn't jam.
    }

    @Override
    public void end(boolean interrupted){
        m_timer.stop();
    }

    @Override
    public boolean isFinished(){
        return m_timer.get() >= m_durationPush;
    }
}
