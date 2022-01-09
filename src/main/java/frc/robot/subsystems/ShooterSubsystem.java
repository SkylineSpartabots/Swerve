package frc.robot.subsystems;


public class ShooterSubsystem {
 
    private static ShooterSubsystem instance = null;

    /*
    private final double maxInitialVelocity = (insert the equation found through encoder)
    
    private double distanceToHub = LimelightSubsystem.getDistance() or DrivetrainSubsystem.getOddometry()

    private double distanceToTopBucket = distanceToHub * Math.sin(some angle, depending on height of shooter and the actual height of hub)
    private double distanceToLowerBucket = distanceToHub * Math.sin(some angle, dependon on height of shooter and the actual height of hub)

    */

    

    public static ShooterSubsystem getInstance(){
        if(instance == null){
            instance = new ShooterSubsystem();
        }
        return instance;
    }



    
}
