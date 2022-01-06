package src.main.java.frc.robot.subsystems;


// FIXME Add needed import statements
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    private String[] getVarList = {"tv", "tx", "ty", "ta", "ts", "tl", "tshort", "tlong", "thor", "tvert", "getpipe", "camtran"};
    
    public Object getVar(String varIn) {
        for (String var : this.getVarList) {
            if (var == varIn) {
                return NetworkTableInstance.getDefault().getTable("limelight").getEntry(varIn).getDouble(0);
            }
        }
    }

final ShuffleboardTab tab = Shuffleboard.getTab("Limelight");

    public void periodic() {

    }
    
}
