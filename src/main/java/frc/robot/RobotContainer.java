// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TurnConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Button;

import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private LimelightSubsystem m_limelight;
  private final XboxController m_controller = new XboxController(0);

  private double previousXSpeed;
  private double previousYSpeed;
  private double previousRotSpeed;

  private final ProfiledPIDController m_thetaController;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //For CAS PID Profiled alignment
    m_thetaController = new ProfiledPIDController(TurnConstants.kPThetaController, TurnConstants.kIThetaController, TurnConstants.kDThetaController, TurnConstants.kThetaControllerConstraints);
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    previousXSpeed = 0;
    previousYSpeed = 0;
    previousRotSpeed = 0;

    m_drivetrainSubsystem = DrivetrainSubsystem.getInstance();
    m_limelight = LimelightSubsystem.getInstance();
    m_limelight.init();

    m_drivetrainSubsystem.zeroGyroscope();
    // Set the scheduler to log Shuffleboard events for command initialize, interrupt, finish
    CommandScheduler.getInstance()
     .onCommandInitialize(
         command ->
             Shuffleboard.addEventMarker(
                 "Command initialized", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command ->
                Shuffleboard.addEventMarker(
                    "Command interrupted", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
        .onCommandFinish(
            command ->
                Shuffleboard.addEventMarker(
                    "Command finished", command.getName(), EventImportance.kNormal));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new Button(m_controller::getBackButton).whenPressed(m_drivetrainSubsystem::zeroGyroscope);
     new Button(m_controller::getAButton).whenPressed(this::resetOdometryFromPosition);
    // new Button(m_controller::getXButton).whenPressed();
    // new Button(m_controller::getYButton).whenPressed(robot::robotInit);
  }

  public DrivetrainSubsystem getDriveTrainSubsystem()
  {
    return m_drivetrainSubsystem;
  }

  public void driveWithJoystick() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -modifyAxis(m_controller.getY(GenericHID.Hand.kLeft)) * DrivetrainSubsystem.MaxSpeedMetersPerSecond;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -modifyAxis(m_controller.getX(GenericHID.Hand.kLeft)) * DrivetrainSubsystem.MaxSpeedMetersPerSecond;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    var rot = -modifyAxis(m_controller.getX(GenericHID.Hand.kRight)) * DrivetrainSubsystem.MaxAngularSpeedRadiansPerSecond;

    if(modifyAxis(m_controller.getX(GenericHID.Hand.kRight))==0){
        
      double tx = findAngle(m_drivetrainSubsystem.getPose(), m_drivetrainSubsystem.getPose().getRotation().getDegrees(), 1, 0);
      double heading_error = tx;
      double Kp = 0.1;
      //double maxSpeed = 2;
      double steering_adjust = Kp*heading_error;
      rot = steering_adjust; //set kp to 0.1
      //rot=Math.copySign(Math.pow(Math.abs(steering_adjust), 0.75),steering_adjust);//multiplies it by the root of the heading error, keeping sign
      //rot = Math.abs(steering_adjust)>maxSpeed?maxSpeed:steering_adjust;      

      //double theta = m_thetaController.calculate(m_drivetrainSubsystem.getPose().getRotation().getRadians(), heading_error);
      //rot = theta;
    }
    
    //apply constraints for acceleration and decceleration

    double deltaXVelocity = xSpeed-previousXSpeed;
    double deltaYVelocity = ySpeed-previousYSpeed;
    double deltaRotVelocity = Math.abs(rot)-Math.abs(previousRotSpeed);//only controls acceleration but not decceleration
    //double deltaRotVelocity = (rot)-(previousRotSpeed);
    double newXSpeed = xSpeed;
    double newYSpeed = ySpeed;
    double newRotSpeed = rot;

    if(Math.abs(deltaXVelocity) > DriveConstants.DriveMaxAccelerationPerPeriodic ){
      newXSpeed = previousXSpeed + Math.copySign(DriveConstants.DriveMaxAccelerationPerPeriodic, deltaXVelocity);
    }
    
    if(Math.abs(deltaYVelocity) > DriveConstants.DriveMaxAccelerationPerPeriodic ){
      newYSpeed = previousYSpeed + Math.copySign(DriveConstants.DriveMaxAccelerationPerPeriodic, deltaYVelocity);
    }
    
    if(deltaRotVelocity > DriveConstants.RotationMaxAccelerationPerPeriodic ){
      newRotSpeed = previousRotSpeed + Math.copySign(DriveConstants.RotationMaxAccelerationPerPeriodic, rot-previousRotSpeed);
    }
    
    previousXSpeed = newXSpeed;
    previousYSpeed = newYSpeed;
    previousRotSpeed = newRotSpeed;


    m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(newXSpeed, newYSpeed, newRotSpeed, m_drivetrainSubsystem.getGyroscopeRotation()));
  }

  public static void resetOdometryFromLimelight(){
      //
  }
  public void resetOdometryFromPosition(){
     m_drivetrainSubsystem.resetOdometry(new Pose2d());
  }

  
  
  public static double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }
  public double findAngle(Pose2d currentPose, double heading, double toX, double toY){
    double deltaY = (toY - currentPose.getY());
    double deltaX = (toX - currentPose.getX());
    double absolute = Math.toDegrees(Math.atan2(deltaY, deltaX));
    double angle = currentPose.getRotation().getDegrees();

    double result = absolute - angle;
    if(Math.abs(result)>180){
      result = -Math.copySign(360-Math.abs(result), result);
    }
    SmartDashboard.putNumber("currentY",currentPose.getY());
    SmartDashboard.putNumber("currentX",currentPose.getX());
    SmartDashboard.putNumber("deltaY",deltaY);
    SmartDashboard.putNumber("deltaX",deltaX);
    SmartDashboard.putNumber("odo rotation",currentPose.getRotation().getDegrees());
    SmartDashboard.putNumber("navX rotation",heading);  
    SmartDashboard.putNumber("absolute angle",absolute);
    SmartDashboard.putNumber("findAngle",result);      

    return  result;
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = applyDeadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
