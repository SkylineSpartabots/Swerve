// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import static frc.robot.Constants.*;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;


public class TurnFromOffset extends CommandBase {
  //@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivetrainSubsystem m_subsystem;

  private final double m_durationInSec;
  private final ProfiledPIDController m_thetaController;

  private final Timer m_timer = new Timer();

  public TurnFromOffset(double p_durationInSec) 
  {
    m_subsystem = DrivetrainSubsystem.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);

    m_durationInSec = p_durationInSec;
    this.withName(m_durationInSec + "_Sec");

    m_thetaController = new ProfiledPIDController(TurnConstants.kPThetaController, TurnConstants.kIThetaController, TurnConstants.kDThetaController, TurnConstants.kThetaControllerConstraints);
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    var currentRotation = m_subsystem.getPose().getRotation();
    double thetaFF = m_thetaController.calculate(currentRotation.getRadians(), currentRotation.getRadians()-LimelightSubsystem.getInstance().getXOffset());

    double steering_adjust = 0.2 * (-LimelightSubsystem.getInstance().getXOffset());
    var targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        0, 0, steering_adjust, currentRotation);
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
    return m_timer.hasElapsed(m_durationInSec);
  }
}
