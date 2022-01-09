package frc.robot.commands;

import frc.robot.subsystems.*;
import static frc.robot.Constants.*;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj2.command.Command;


public class DriveCommandFactory {

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public static Command getAutonomousCommand() {

    var command =
      new TrajectoryDriveCommand(
          List.of(
            new Translation2d(1, 0), 
            new Translation2d(2, 1)
          ),
          new Pose2d(3, 1, new Rotation2d(0)),
          false);
    return command;
  }

  // use field relative coordinate, move to target.
  public static Command getDriveToCommand(double p_targetX, double p_targetY, double p_targetAngleRad)
  {
    return new TrajectoryDriveCommand(List.of(), new Pose2d(p_targetX, p_targetY, new Rotation2d(p_targetAngleRad)), true); 
  }

  public static Command get2019AutonomousCommandSimple() {

    var subsystem = DrivetrainSubsystem.getInstance();
    // set current position as (x=0m, y=0m, angle=0Radian)
    subsystem.zeroGyroscope();
    subsystem.resetOdometry(new Pose2d(0,0, new Rotation2d(0)));

    var startPose = subsystem.getPose();
    var wayPoints = List.of(
      new Translation2d(-1, -1), 
      new Translation2d(-3, -1),
      new Translation2d(-3.5, -1),
      new Translation2d(-2.5, -1),
      new Translation2d(-2, -1));
    var endPose = new Pose2d(0, 0, new Rotation2d(0));

    var config = new TrajectoryConfig(
      DrivetrainSubsystem.MaxSpeedMetersPerSecond, 
      DrivetrainSubsystem.MaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);

    var trajectory = TrajectoryGenerator.generateTrajectory(startPose, wayPoints, endPose, config);

    var command = new TrajectoryFollowCommand(trajectory, true /*enablePID*/);
    return command;
  }  

  public static Command get2019AutonomousCommandComplex() {

    var subsystem = DrivetrainSubsystem.getInstance();
    // set current position as (x=0m, y=0m, angle=0Radian)
    subsystem.zeroGyroscope();
    subsystem.resetOdometry(new Pose2d(0,0, new Rotation2d(0)));

    var startPose = subsystem.getPose();
    var wayPoints0 = List.of(new Translation2d(-1, -1));
    var endPose0 = new Pose2d(-1, -1, new Rotation2d(0));

    var config = new TrajectoryConfig(
      DrivetrainSubsystem.MaxSpeedMetersPerSecond, 
      DrivetrainSubsystem.MaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics)
    .setEndVelocity(1); // not stop, keep velocity at end
    var trajectory0 = TrajectoryGenerator.generateTrajectory(startPose, wayPoints0, endPose0, config);

    var wayPoints1 = List.of(new Translation2d(-2, -1));
    var endPose1 = new Pose2d(-3, -1, new Rotation2d(0));
    config.setStartVelocity(1).setEndVelocity(.5); 
    var trajectory1 = TrajectoryGenerator.generateTrajectory(endPose0, wayPoints1, endPose1, config);

    var wayPoints2 = List.of(new Translation2d(-3.2, -1));
    var endPose2 = new Pose2d(-3.5, -1, new Rotation2d(0));
    config.setStartVelocity(.5).setEndVelocity(0); 
    var trajectory2 = TrajectoryGenerator.generateTrajectory(endPose1, wayPoints2, endPose2, config);

    var wayPoints3 = List.of(new Translation2d(-2.5, -1), new Translation2d(-2, -1));
    var endPose3 = new Pose2d(0, 0, new Rotation2d(0));
    config.setStartVelocity(0).setEndVelocity(0); 
    var trajectory3 = TrajectoryGenerator.generateTrajectory(endPose2, wayPoints3, endPose3, config);

    var trajectory = trajectory0;
    trajectory.concatenate(trajectory1);
    trajectory.concatenate(trajectory2);
    trajectory.concatenate(trajectory3);
    var command = new TrajectoryFollowCommand(trajectory, true /*enablePID*/);
    return command;
  }  
}