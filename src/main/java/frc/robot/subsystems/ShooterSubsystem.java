package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.lang.Math;

import java.lang.module.ModuleDescriptor.Modifier;


public class ShooterSubsystem extends SubsystemBase {
    public ShooterSubsystem(double desiredDistance) {

    }
    
    private final TalonFX flywheelTalon = new Talon(Constants.SHOOTER_FLYWHEEL_MOTOR);

    private boolean ballShotState;
    private double ballShotTime;
    private double flywheelVelo = flywheelTalon.getSelectedSensorVelocity(0);

    private double rawVeloToRpm(double velo) {
        return velo / Constants.FALCON_VELO_RPM_FACTOR;
    }
    
    private double rpmtoRadSec(double rpm) {
        return Units.rotationsPerMinuteToRadiansPerSecond(rpm);
    }

    private double rpmToBallVeloX() {
        return (this.rpmtoRadSec(rawVeloToRpm(flywheelVelo))*Math.cos(Units.degreesToRadians(Constants.SHOOTER_FLYWHEEL_ANGLE_DEGREES)-9.81*this.ballShotTime)
    }
    
    private double rpmToBallVeloY() {
        return (this.rpmtoRadSec(rawVeloToRpm(flywheelVelo))*Math.sin(Constants.SHOOTER_FLYWHEEL_ANGLE_DEGREES)-9.81*this.ballShotTime);
    }

    @Override
    public void periodic() {
        if (ballShotState) {
            ballShotTime = ballShotTime+0.02;
        } else {
            ballShotTime = 0;
        }
        
    }

    @Override
    public void simulationPeriodic() {
        this.periodic();
    }
    
    
}
