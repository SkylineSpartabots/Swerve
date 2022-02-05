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

public class IndexerSubsystem extends SubsystemBase{

    private static IndexerSubsystem instance = null;

	//hardware
    private final LazyTalonFX m_IndexerMotor;

    public static IndexerSubsystem getInstance(){
        if(instance == null){
            instance = new IndexerSubsystem();
        }
        return instance;
    }

    private IndexerSubsystem(){
        IndexerMotor = TalonFXFactory.createDefaultFalcon("Indexer Motor", Constants.IndexerConstants.INDEXER_MOTOR);
        configureShooterMotor(IndexerMotor, false);
    }

    private void configureShooterMotor (LazyTalonFX talon, boolean inversion){
        talon.setInverted(inversion);
        PheonixUtil.checkError(talon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs),
            talon.getName() + " failed to set voltage compensation", true);
        talon.enableVoltageCompensation(true);
        talon.setNeutralMode(NeutralMode.Coast);
    }

    public void setIndexerPowerPercent(double speed){
        IndexerMotor.set(ControlMode.PercentOutput, speed);
    }

	private ShuffleboardTab debugTab = Shuffleboard.getTab("Indexer");
    private NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
    private NetworkTable table = tableInstance.getTable("Indexer");

    @Override
    public void periodic() {
        table.getEntry("Indexer Supply Current").setDouble(m_IndexerMotor.getSupplyCurrent());
        table.getEntry("Indexer Stator Current").setDouble(m_IndexerMotor.getStatorCurrent());
        table.getEntry("Indexer Supply Current").setDouble(m_IndexerMotor.getLastSet());
    }

    @Override
    public void simulationPeriodic() {
        this.periodic();
    }
}
