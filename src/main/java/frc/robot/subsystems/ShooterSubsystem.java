package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
    private ShuffleboardTab debugTab = Shuffleboard.getTab("Shooter");

    public static ShooterSubsystem getInstance() {
        if (instance == null) {
            instance = new ShooterSubsystem();
        }
        return instance;
    }

    private double rawVeloToRpm(double velo) {
        return velo / ShooterConstants.kFalconVeloRpmFactor;
    }

    private double rpmtoRadSec(double rpm) {
        return Units.rotationsPerMinuteToRadiansPerSecond(rpm);
    }

    private void setMotorPower(double power) {
        flywheelTalon.set(ControlMode.PercentOutput, power);
    }

    @Override
    public void periodic() {
        debugTab.add((Sendable) flywheelTalon);
    }

    @Override
    public void simulationPeriodic() {
        this.periodic();
    }
}
