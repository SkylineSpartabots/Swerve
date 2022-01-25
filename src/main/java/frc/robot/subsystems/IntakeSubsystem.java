package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.LazyTalonSRX;
import frc.lib.drivers.PheonixUtil;
import frc.lib.drivers.TalonSRXUtil;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class IntakeSubsystem extends SubsystemBase {

    private static IntakeSubsystem instance = null;

    //debug
    private final boolean debug = false;
    private ShuffleboardTab debugTab = Shuffleboard.getTab("Intake");

    //hardware
    private final LazyTalonFX mInnerIntakeMotor, mOuterIntakeMotor;

    //control states
    private boolean mStateChanged = false;
    private double mStateChangeTimestamp = 0;

    public static IntakeSubsystem getInstance() {
        if (instance == null) {
            instance = new IntakeSubsystem();
        }
        return instance;
    }

    public IntakeSubsystem() {
        mInnerIntakeMotor = null; // FIXME -  don't have ports, so im setting them to null for now
        mOuterIntakeMotor = null; // good job build
    }

    private void configureMotor(LazyTalonSRX mTalon, InvertType inversion) {
        mTalon.setInverted(inversion);
        PheonixUtil.checkError(mTalon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs),
                mTalon.getName() + " failed to set voltage compensation", true);
        PheonixUtil.checkError(mTalon.configVoltageMeasurementFilter(32, Constants.kTimeOutMs),
                mTalon.getName() + " failed to set voltage meas. filter", true);
        mTalon.enableVoltageCompensation(true);

        TalonSRXUtil.setCurrentLimit(mTalon, 25);

        mTalon.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public void periodic() {
        debugTab.add("Test", "Test");
    }

    @Override
    public void simulationPeriodic() {
        this.periodic();
    }
}
