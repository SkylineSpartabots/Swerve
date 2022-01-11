package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    private static ShooterSubsystem instance = null;

    /*
    private final double maxInitialVelocity = (insert the equation found through encoder)

    private double distanceToHub = LimelightSubsystem.getDistance() or DrivetrainSubsystem.getOddometry()

    private double distanceToTopBucket = distanceToHub * Math.sin(some angle, depending on height of shooter and the actual height of hub)
    private double distanceToLowerBucket = distanceToHub * Math.sin(some angle, dependon on height of shooter and the actual height of hub)

    */
    private static double desiredDistance = 0;
    private final TalonFX flywheelTalon = new TalonFX(Constants.SHOOTER_FLYWHEEL_MOTOR);
    private ShuffleboardTab debugTab = Shuffleboard.getTab("Shooter");
    private boolean ballShotState;
    private double ballShotTime;
    private double flywheelVelo = flywheelTalon.getSelectedSensorVelocity(0);
    private double flywheelRpm = rawVeloToRpm(flywheelVelo);
    public ShooterSubsystem(double desiredDistance) {
        desiredDistance = ShooterSubsystem.desiredDistance;
    }

    public static ShooterSubsystem getInstance() {
        if (instance == null) {
            instance = new ShooterSubsystem(desiredDistance);
        }
        return instance;
    }

    private double rawVeloToRpm(double velo) {
        return velo / Constants.FALCON_VELO_RPM_FACTOR;
    }

    private double rpmtoRadSec(double rpm) {
        return Units.rotationsPerMinuteToRadiansPerSecond(rpm);
    }

    private double rpmToBallVeloX() {
        return (this.rpmtoRadSec(
                rawVeloToRpm(flywheelVelo))
                * Math.cos(Units.degreesToRadians(
                Constants.SHOOTER_FLYWHEEL_ANGLE_DEGREES
        ) - 9.81 * this.ballShotTime));
    }

    private double rpmToBallVeloY() {
        return (this.rpmtoRadSec
                (rawVeloToRpm(
                        flywheelVelo
                ))
                * Math.sin(Constants.SHOOTER_FLYWHEEL_ANGLE_DEGREES)
                - 9.81 * this.ballShotTime);
    }

    private void ballShotTimer() {
        if (ballShotState) {
            ballShotTime = ballShotTime + 0.02;
        } else {
            ballShotTime = 0;
        }
    }

    @Override
    public void periodic() {
        this.ballShotTimer();
        debugTab.add("Flywheel RPM", flywheelRpm);
        debugTab.add("Flywheel Rad/sec", this.rpmtoRadSec(flywheelRpm));
        debugTab.add("Ball X Velocity", this.rpmToBallVeloX());
        debugTab.add("Ball Y Velocity", this.rpmToBallVeloY());
    }

    @Override
    public void simulationPeriodic() {
        this.periodic();
    }
}
