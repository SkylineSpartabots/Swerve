/*
package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class BeambreakSubsystem implements Subsystem{
    private static BeambreakSubsystem m_instance = null;
    private static final int BeambreakPort = 0;
    private boolean ballInIndexer;
    private boolean secondBallInIndexer;

    public static BeambreakSubsystem getInstance(){
        if (m_instance == null) {
              m_instance = new BeambreakSubsystem();
        }

        return m_instance;
    }

    private final DigitalInput beambreakSensor;

    public BeambreakSubsystem(){
        beambreakSensor = new DigitalInput(BeambreakPort);
        ballInIndexer = false;
        secondBallInIndexer = false;
    }

    //potentially override and change value of ball in indexer after the ball is shot, will need to be called by commands, in the end().
    //might not be needed.
    public void OneBallShot(){

    }

    public void BothBallShot(){

    }

    public void updateSensor(){
        ballInIndexer = !beambreakSensor.get();

    }


    private ShuffleboardTab debugTab = Shuffleboard.getTab("Beambreak");
    private NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
    private NetworkTable table = tableInstance.getTable("Beambreak");

    @Override
    public void periodic(){
        table.getEntry("Ball intaked").setBoolean(ballInIndexer);
        ballInIndexer = !beambreakSensor.get();
        //this is one way to implement this, having the value be updated periodically within subsystem, TODO: look at best practice for updating and getting sensor value.
    }
}
*/
//do not use this, beam breaks are very unstable and can be triggered by vibration or getting hit - Mustafa.