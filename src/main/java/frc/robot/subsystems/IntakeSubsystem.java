package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.PheonixUtil;
import frc.lib.drivers.TalonFXFactory;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class IntakeSubsystem extends SubsystemBase {

    private static IntakeSubsystem instance = null;

    //hardware
    private final LazyTalonFX m_IntakeMotor;

    public static IntakeSubsystem getInstance() {
        if (instance == null) {
            instance = new IntakeSubsystem();
        }
        return instance;
    }

    public IntakeSubsystem() {
        m_IntakeMotor = TalonFXFactory.createDefaultFalcon("Shooter Motor", Constants.IntakeConstants.INTAKE_MOTOR_PORT);
		configureMotor(m_IntakeMotor, InvertType.None);
    }

    private void configureMotor(LazyTalonFX m_IntakeMotor2, InvertType inversion) {
        m_IntakeMotor2.setInverted(inversion);
        PheonixUtil.checkError(m_IntakeMotor2.configVoltageCompSaturation(12.0, Constants.kTimeOutMs),
                m_IntakeMotor2.getName() + " failed to set voltage compensation", true);
        m_IntakeMotor2.enableVoltageCompensation(true);
        m_IntakeMotor2.setNeutralMode(NeutralMode.Coast);
    }

    public void setIntakeSpeed(double speed) {
        m_IntakeMotor.set(ControlMode.PercentOutput, speed);
    }

    private ShuffleboardTab debugTab = Shuffleboard.getTab("Intake");
    private NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
    private NetworkTable table = tableInstance.getTable("Intake");

    @Override
    public void periodic() {
        table.getEntry("Intake Supply Current").setDouble(m_IntakeMotor.getSupplyCurrent());
        table.getEntry("Intake Stator Current").setDouble(m_IntakeMotor.getStatorCurrent());
        table.getEntry("Intake Supply Current").setDouble(m_IntakeMotor.getLastSet());
    }

    @Override
    public void simulationPeriodic() {
        this.periodic();
    }
}
