package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

	private final IntakeSubsystem m_subsystem = IntakeSubsystem.getInstance();

	private final Timer m_timer = new Timer();
	private final double m_durationOn = 0.75;
	private double intakeSpeed;

	public IntakeCommand(double speed) {
		addRequirements(m_subsystem);
		intakeSpeed = speed;
	}

	@Override
	public void initialize() {
		m_timer.reset();
		m_timer.start();
	}

	@Override
	public void execute() {
		m_subsystem.setIntakeSpeed(intakeSpeed);
	}

	@Override
	public void end(boolean interrupted) {
		m_timer.stop();
		m_subsystem.setIntakeSpeed(Constants.IntakeConstants.kStopIntake);
	}

	@Override
	public boolean isFinished() {
		return m_timer.get() > m_durationOn;
	}

	
}
