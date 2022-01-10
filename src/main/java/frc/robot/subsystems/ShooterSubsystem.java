package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import java.lang.Math;

public class ShooterSubsystem extends SubsystemBase {

    private static ShooterSubsystem instance = null;

    /*
     * private final double maxInitialVelocity = (insert the equation found through
     * encoder)
     * 
     * private double distanceToHub = LimelightSubsystem.getDistance() or
     * DrivetrainSubsystem.getOddometry()
     * 
     * private double distanceToTopBucket = distanceToHub * Math.sin(some angle,
     * depending on height of shooter and the actual height of hub)
     * private double distanceToLowerBucket = distanceToHub * Math.sin(some angle,
     * dependon on height of shooter and the actual height of hub)
     * 
     */

    public static ShooterSubsystem getInstance() {
        if (instance == null) {
            instance = new ShooterSubsystem(desiredDistance);
        }
        return instance;
    }

    private ShuffleboardTab debugTab = Shuffleboard.getTab("Shooter");

    private final TalonFX flywheelTalon = new TalonFX(ShooterConstants.SHOOTER_FLYWHEEL_MOTOR);

    private static double desiredDistance = 0;
    private boolean ballShotState;
    private double ballShotTime;
    private double flywheelVelo = flywheelTalon.getSelectedSensorVelocity(0);
    private double flywheelRpm = rawVeloToRpm(flywheelVelo);

    public ShooterSubsystem(double desiredDistance) {
        desiredDistance = ShooterSubsystem.desiredDistance;
    }

    private double rawVeloToRpm(double velo) {
        return velo / ShooterConstants.FALCON_VELO_RPM_FACTOR;
    }

    private double rpmtoRadSec(double rpm) {
        return Units.rotationsPerMinuteToRadiansPerSecond(rpm);
    }

    private double rpmToBallVeloX(double time) {
        double velo = this.flywheelRpm;
        velo = velo * (Math.cos(Units.degreesToRadians(ShooterConstants.SHOOTER_FLYWHEEL_ANGLE_DEGREES)));
        velo = velo - 9.81;
        velo = velo * time;
        return velo;
    }

    private double rpmToBallVeloY(double time) {
        double velo = this.flywheelRpm;
        velo = velo * (Math.sin(Units.degreesToRadians(ShooterConstants.SHOOTER_FLYWHEEL_ANGLE_DEGREES)));
        velo = velo - 9.81;
        velo = velo * time;
        return velo;
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
        debugTab.add("Ball X Velocity", this.rpmToBallVeloX(this.ballShotTime));
        debugTab.add("Ball Y Velocity", this.rpmToBallVeloY(this.ballShotTime));
    }

    @Override
    public void simulationPeriodic() {
        this.periodic();
    }
}
