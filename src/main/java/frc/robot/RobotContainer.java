// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

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


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

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
    Button xButton = new Button(m_controller::getXButton);
    Trigger xAndRStickTrigger = new Trigger(){
      @Override
      public boolean get(){
        return xButton.get() && m_controller.getStickButton(Hand.kRight);
      }
    };
    xButton.whenPressed(this::softResetOdometryFromReference);
    xAndRStickTrigger.whenActive(this::hardResetOdometryFromReference);
  }

  public DrivetrainSubsystem getDriveTrainSubsystem(){
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
      /*
      double tx = LimelightSubsystem.getInstance().getXOffset();
      double heading_error = -tx;
      double Kp = 0.2;
      double min_command = 0.05;
        double steering_adjust = 0.0f;
        if (tx > 1.0)
        {
                steering_adjust = Kp*heading_error - min_command;
        }
        else if (tx < 1.0)
        {
                steering_adjust = Kp*heading_error + min_command;
        }
      rot = steering_adjust;
      */

        
      double tx = findAngle(m_drivetrainSubsystem.getPose(), 1, 0);
      double heading_error = -tx;
      double Kp = 0.2;
      double maxSpeed = 2;
      double steering_adjust = Kp* //takes the kp constant
        Math.copySign(Math.pow(Math.abs(heading_error), 0.25),heading_error);//multiplies it by the root of the heading error, keeping sign
      //rot = steering_adjust>maxSpeed?maxSpeed:steering_adjust;
      SmartDashboard.putNumber("Tx", tx);
      SmartDashboard.putNumber("Steering", steering_adjust);

      Translation2d limelightEstimation = m_limelight.getFieldPositionFromTarget(m_drivetrainSubsystem.getGyroscopeRotation().getRadians());
    }
    
    m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_drivetrainSubsystem.getGyroscopeRotation()));
  }

  public static void resetOdometryFromLimelight(){
      //
  }
  public void resetOdometryFromPosition(){
     m_drivetrainSubsystem.resetOdometry(new Pose2d());
  }

  public void resetOdometryFromReference(double threshold){
    Translation2d current = m_drivetrainSubsystem.getPose().getTranslation();
    double minError = FieldConstants.kMinReferenceError;
    Translation2d newPos = null;
    for(Translation2d ref : FieldConstants.kReferenceTranslations){
      double errorX = Math.abs(ref.getX() - current.getX());
      double errorY = Math.abs(ref.getY() - current.getY());
      double error = Math.min(minError, Math.sqrt(Math.pow(errorX, 2) + Math.pow(errorY, 2)));
      if(error < minError){
        newPos = ref;
        minError = error;
      }
    }
    if(minError < FieldConstants.kMinReferenceError){
      m_drivetrainSubsystem.resetOdometry(new Pose2d(newPos, m_drivetrainSubsystem.getGyroscopeRotation()));
      SmartDashboard.putBoolean("Too Far From Reference", false);
    }
    else
      SmartDashboard.putBoolean("Too Far From Reference", true);
  }

  public void softResetOdometryFromReference(){
    resetOdometryFromReference(FieldConstants.kMinReferenceError);
  }
  public void hardResetOdometryFromReference(){
    resetOdometryFromReference(100d);
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
  public static double findAngle(Pose2d currentPose, double toX, double toY){
    double deltaY = (toY - currentPose.getY());
    double deltaX = (toX - currentPose.getX());
    SmartDashboard.putNumber("CurrX", currentPose.getX());
    SmartDashboard.putNumber("CurrY", currentPose.getY());
    double rot = currentPose.getRotation().getDegrees();
    if(rot > 180)
      rot -= 180;
    else if (rot > -180 && rot < 180)
      rot += 180;
      //Unit circle is NOW: awsd
      //0: +x
      //90: +y
      //180: -x
      //270: -y
      //It loops between 0 and 360 degrees
    SmartDashboard.putNumber("CurrentRot", rot);
    return (Math.toDegrees(Math.atan2(deltaY, deltaX)) - currentPose.getRotation().getDegrees());
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = applyDeadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
