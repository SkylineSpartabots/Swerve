// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import static frc.robot.Constants.*;
import frc.robot.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class TrapezoidProfileTurnCommand extends CommandBase {
  private final DrivetrainSubsystem m_subsystem;

  private final ProfiledPIDController m_thetaController;
  private final TrapezoidProfile m_profile;

  private final Timer m_timer = new Timer();

  public TrapezoidProfileTurnCommand(
    double p_endRadians) 
  {
    m_subsystem = DrivetrainSubsystem.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);

    m_thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    m_profile = new TrapezoidProfile(AutoConstants.kThetaControllerConstraints, new TrapezoidProfile.State(p_endRadians, 0));

    this.withName("TrapezoidProfileTurnCommand_" + p_endRadians +":totalTime" + m_profile.totalTime() + "");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    var state = m_profile.calculate(m_timer.get());
    var thetaFF = state.velocity;

    var currentRotation = m_subsystem.getPose().getRotation();
    double thetaFeedBack = m_thetaController.calculate(currentRotation.getRadians(), state.position);

    var targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        0, 0, thetaFF+thetaFeedBack, currentRotation);
    m_subsystem.drive(targetChassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_subsystem.drive(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_profile.totalTime());
  }
}
