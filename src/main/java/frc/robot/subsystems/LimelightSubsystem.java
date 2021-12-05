package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

    public LimelightSubsystem(){
        
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);

    }
    
}
