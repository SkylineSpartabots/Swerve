// FIXME Add needed import statements

public class LimelightCommand extends CommandBase {
    private LimelightSubsystem m_limelight;

    public LimelightCommand(LimelightSubsystem subsystem) {
        m_limelight = subsystem;
        addRequirements(m_limelight);
    }

    public void initialize() {
        
    }
}