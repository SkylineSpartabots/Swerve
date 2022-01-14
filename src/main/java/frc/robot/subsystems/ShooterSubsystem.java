package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

public class ShooterSubsystem extends SubsystemBase {
    private static ShooterSubsystem instance = null;
    /*
    private final double maxInitialVelocity = (insert the equation found through encoder)

    private double distanceToHub = LimelightSubsystem.getDistance() or DrivetrainSubsystem.getOddometry()

    private double distanceToTopBucket = distanceToHub * Math.sin(some angle, depending on height of shooter and the actual height of hub)
    private double distanceToLowerBucket = distanceToHub * Math.sin(some angle, dependon on height of shooter and the actual height of hub)

    */
    private final TalonFX flywheelTalon = new TalonFX(ShooterConstants.kFlywheelMotor);

    public static ShooterSubsystem getInstance() {
        if (instance == null) {
            instance = new ShooterSubsystem();
        }
        return instance;
    }

    private double rawVeloToRpm(double velo) {
        return velo / ShooterConstants.kFalconVeloRpmFactor;
    }

    public void setMotorPower(double power) {
        flywheelTalon.set(ControlMode.PercentOutput, power);
    }
    
    private ShuffleboardTab debugTab = Shuffleboard.getTab("Shooter");
    private NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
    private NetworkTable table = tableInstance.getTable("Shooter");

    @Override
    public void periodic() {
        table.getEntry("Flywheel Talon Velocity").setDouble(flywheelTalon.getSelectedSensorVelocity());
        table.getEntry("Flywheel Talon Power").setDouble(flywheelTalon.getMotorOutputPercent());
        table.getEntry("Flywheel RPM").setDouble(rawVeloToRpm(flywheelTalon.getSelectedSensorVelocity()));
        table.getEntry("Shooter On?").setBoolean(true);
        table.getEntry("Shooter Subsystem").setValue(ShooterSubsystem.instance);
    }

    @Override
    public void simulationPeriodic() {
        this.periodic();
    }
}
