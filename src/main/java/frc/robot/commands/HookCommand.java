package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class HookCommand extends CommandBase {
    private final ClimbSubsystem m_subsystem;
    private final Timer m_timer = new Timer();

    public HookCommand(ClimbSubsystem subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
        public void initialize() {
            m_timer.reset();
            m_timer.start();
            m_subsystem.raiseHook();
    }

    @Override
        public void execute() {
            if (m_timer.hasElapsed(5.0)) {
                m_subsystem.off();
            }
        }

}