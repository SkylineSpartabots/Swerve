
package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.kauailabs.navx.frc.AHRS;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SwerveDriveByVoltage extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MaxSpeedMetersPerSecond = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4_L2.getDriveReduction() *
          SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

  // need measure on robot
  public static final double MaxAccelerationMetersPerSecondSquared = 10; 

  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MaxAngularSpeedRadiansPerSecond = MaxSpeedMetersPerSecond /
          Math.hypot(DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0);

  private final SwerveDriveOdometry m_odometry;

  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private double m_voltage;

  private static SwerveDriveByVoltage m_instance = null;
  public static SwerveDriveByVoltage getInstance(){
          if (m_instance == null) {
                m_instance = new SwerveDriveByVoltage();
          }

          return m_instance;
  }

  public SwerveDriveByVoltage() {
    ShuffleboardTab tab = Shuffleboard.getTab("SwerveDriveByVoltage");

    m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk4SwerveModuleHelper.GearRatio.L2,
            // This is the ID of the drive motor
            DriveConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            DriveConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            DriveConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            DriveConstants.FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // We will do the same for the other modules
    m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4SwerveModuleHelper.GearRatio.L2,
            DriveConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            DriveConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
            DriveConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
            DriveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk4SwerveModuleHelper.GearRatio.L2,
            DriveConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
            DriveConstants.BACK_LEFT_MODULE_STEER_MOTOR,
            DriveConstants.BACK_LEFT_MODULE_STEER_ENCODER,
            DriveConstants.BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk4SwerveModuleHelper.GearRatio.L2,
            DriveConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            DriveConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
            DriveConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
            DriveConstants.BACK_RIGHT_MODULE_STEER_OFFSET
    );

    m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getGyroscopeRotation());
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    m_navx.zeroYaw();
  }

  public Rotation2d getGyroscopeRotation() {
   if (m_navx.isMagnetometerCalibrated()) {
     // We will only get valid fused headings if the magnetometer is calibrated
     return Rotation2d.fromDegrees(m_navx.getFusedHeading());
   }
   // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
   return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }
  
  public Pose2d getPose(){
        return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d p_pose) {
    m_odometry.resetPosition(p_pose, getGyroscopeRotation());
  }

  @Override
  public void periodic() {

    m_frontLeftModule.set(m_voltage, 0);
    m_frontRightModule.set(m_voltage, 0);
    m_backLeftModule.set(m_voltage, 0);
    m_backRightModule.set(m_voltage, 0);
        
    m_odometry.update(getGyroscopeRotation(), 
      new SwerveModuleState(m_frontLeftModule.getDriveVelocity(), new Rotation2d(m_frontLeftModule.getSteerAngle())),
      new SwerveModuleState(m_frontRightModule.getDriveVelocity(), new Rotation2d(m_frontRightModule.getSteerAngle())),
      new SwerveModuleState(m_backLeftModule.getDriveVelocity(), new Rotation2d(m_backLeftModule.getSteerAngle())),
      new SwerveModuleState(m_backRightModule.getDriveVelocity(), new Rotation2d(m_backRightModule.getSteerAngle())));

    var pose = m_odometry.getPoseMeters();
    SmartDashboard.putNumber("X Position", pose.getTranslation().getX());
    SmartDashboard.putNumber("Y Position", pose.getTranslation().getY());
    SmartDashboard.putNumber("Rotation", getGyroscopeRotation().getDegrees());
  }

  /*
    Baseline drivetrain logic is assuming target velocity is linear to voltage following 
      Voltage = k * velocity

    In theory, the equation should more like to be
      Voltage = kS * sign(velocity) + kV * velocity + kA * acceleration
    
    For normal case where sign(velocity) == 1, and acceleration == 0, it should be
      Voltage = kS + kV * velocity

    to find out what is the value of kS, we can run test by using constant voltage record the X position, to capture the stable 
    velocity it reached. Running the test using several different voltage like 3V, 6V, 9V and 12V, we should be able to calculate 
    value of kS and kV. 

    In game, we most likely to run the robot either stop or a high speed, we may run more voltage near the high side like 9V, 10V, 11V and 12V.

    Extra: although battery have target voltage of 12V, we may use something lower like 11.5V to get a consistent output even when battery is not full.
  */
  public void DriveByVoltage(double p_voltage)
  {
    m_voltage = p_voltage;
  }
}
