package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.LazyTalonFX;

public class TurretSubsystem extends SubsystemBase {

    private static DrivetrainSubsystem m_instance = null;
    public final int deviceID = 0;
    public final LazyTalonFX shooterMotor = new LazyTalonFX("turret", deviceID);

    public static DrivetrainSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new DrivetrainSubsystem();
        }

        return m_instance;
    }


}
