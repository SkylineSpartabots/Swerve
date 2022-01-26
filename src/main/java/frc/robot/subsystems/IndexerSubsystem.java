package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.PheonixUtil;
import frc.lib.drivers.TalonFXFactory;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase{
    private static IndexerSubsystem instance = null;

    private final LazyTalonFX IndexerMotor;

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

    public void setIndexerPowerPercent(double power){
        IndexerMotor.set(ControlMode.PercentOutput, power);
    }
}
