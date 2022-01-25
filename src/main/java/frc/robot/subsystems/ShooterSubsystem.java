package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.TalonFXFactory;
import frc.lib.drivers.*;
import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.LazyTalonSRX;
import frc.lib.drivers.PheonixUtil;
import frc.lib.drivers.TalonSRXFactory;
import frc.lib.drivers.TalonSRXUtil;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem instance = null;

    /*
    private final double maxInitialVelocity = (insert the equation found through encoder)

    private double distanceToHub = LimelightSubsystem.getDistance() or DrivetrainSubsystem.getOddometry()

    private double distanceToTopBucket = distanceToHub * Math.sin(some angle, depending on height of shooter and the actual height of hub)
    private double distanceToLowerBucket = distanceToHub * Math.sin(some angle, dependon on height of shooter and the actual height of hub)

    */

    private final LazyTalonFX mMasterShooter, mSlaveShooter;
    private final DrivetrainSubsystem mDrivetrainSubsystem = DrivetrainSubsystem.getInstance();

    private boolean mOnTarget;
    
    public static ShooterSubsystem getInstance() {
        if (instance == null) {
            instance = new ShooterSubsystem();
        }
        return instance;
    }

    private ShooterSubsystem() {
        mMasterShooter = TalonFXFactory.createDefaultFalcon("Left Shooter Motor", Constants.ShooterConstants.MASTER_SHOOTER_MOTOR);

        configMasterForShooter(mMasterShooter, InvertType.InvertMotorOutput, true);


        mSlaveShooter = TalonFXFactory.createSlaveFalcon("Right Shooter Motor", Constants.ShooterConstants.SLAVE_SHOOTER_MOTOR, Constants.ShooterConstants.MASTER_SHOOTER_MOTOR);
        configFalconForShooter(mSlaveShooter, InvertType.FollowMaster);
        mSlaveShooter.setMaster(mMasterShooter);

        SmartDashboard.putNumber("Shooter kP", 0.0);
        SmartDashboard.putNumber("Shooter kI", 0.000022);
        SmartDashboard.putNumber("Shooter kD", 0.0);
        SmartDashboard.putNumber("Shooter kF", 0.048000);
    }

    private void configMasterForShooter(LazyTalonFX talon, InvertType inversion, boolean sensorPhase){
        configureShooterMotor(talon, inversion);
    }

    private void configureShooterMotor (LazyTalonFX talon, InvertType inversion){
        talon.setInverted(inversion);
        PheonixUtil.checkError(talon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs),
            talon.getName() + " failed to set voltage compensation", true);
        talon.enableVoltageCompensation(true);
        talon.setNeutralMode(NeutralMode.Coast);
    }

    private void configFalconForShooter(LazyTalonFX falcon, InvertType inversion) {
        falcon.setInverted(inversion);
        falcon.setNeutralMode(NeutralMode.Coast);
    }

    private double rawVeloToRpm(double velo) {
        return velo / ShooterConstants.kFalconVeloRpmFactor;
    }

    public void setMotorPowerVelocity(double power) {
        mMasterShooter.set(ControlMode.Velocity, power/10);
    }

    public void setMotorPowerPercent(double power) {
        mMasterShooter.set(ControlMode.PercentOutput, power);
    }
    
    private ShuffleboardTab debugTab = Shuffleboard.getTab("Shooter");
    private NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
    private NetworkTable table = tableInstance.getTable("Shooter");

    @Override
    public void periodic() {
        table.getEntry("Flywheel Talon Velocity").setDouble(mMasterShooter.getSelectedSensorVelocity());
        table.getEntry("Flywheel Talon Power").setDouble(mMasterShooter.getMotorOutputPercent());
        table.getEntry("Flywheel RPM").setDouble(rawVeloToRpm(mMasterShooter.getSelectedSensorVelocity()));
        table.getEntry("Shooter On?").setBoolean(true);
        table.getEntry("Shooter Subsystem").setValue(ShooterSubsystem.instance);
    }

    @Override
    public void simulationPeriodic() {
        this.periodic();
    }
}