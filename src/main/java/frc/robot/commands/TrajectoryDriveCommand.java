// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import static frc.robot.Constants.*;
import frc.robot.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

import java.util.List;

public class TrajectoryDriveCommand extends CommandBase {
  //@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivetrainSubsystem m_subsystem;

  private TrajectoryConfig m_config;
  private Trajectory m_trajectory;
  private Rotation2d m_endRotation;
  private final HolonomicDriveController m_controller;

  private final Timer m_timer = new Timer();

  public TrajectoryDriveCommand(
    List<Translation2d> p_interiorWaypoints, 
    Pose2d p_end,
    boolean p_enablePID) 
  {
    m_subsystem = DrivetrainSubsystem.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);

    this.withName("DriveTo_" + p_end.toString());

    m_config = new TrajectoryConfig(DrivetrainSubsystem.MaxSpeedMetersPerSecond, DrivetrainSubsystem.MaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(DriveConstants.kDriveKinematics);
    
    m_trajectory = TrajectoryGenerator.generateTrajectory(m_subsystem.getPose(), p_interiorWaypoints, p_end, m_config);
    m_endRotation = p_end.getRotation();
    var xController = new PIDController(AutoConstants.kPXController, 0, 0);
    var yController = new PIDController(AutoConstants.kPYController, 0, 0);     
    var thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    m_controller = new HolonomicDriveController(xController, yController, thetaController);
    m_controller.setEnabled(p_enablePID);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    double curTime = m_timer.get();
    var desiredState = m_trajectory.sample(curTime);

    var targetChassisSpeeds = m_controller.calculate(m_subsystem.getPose(), desiredState, m_endRotation);
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
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }
}
