package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;

public class DoubleShootCommand extends SequentialCommandGroup{

    private final double timeForLaunch = 1.5;
    public DoubleShootCommand(double distance){
        addCommands(
            new SingleShootCommand(distance),
            //new indexerPush(),
            new SingleShootCommand(distance),
            //new indexerPush(),
            new ShooterConstantPowerCommand()
        );/*indexerPush is simply to push the ball into contact with the motor,
        motors will continue moving at velocity speed set by SingleShootCommand(), and will return to normal with ShooterConstantPowerCommand().
        This could run into issues with multiple ShooterConstantPowerCommands layered, will need testing*/
    }
}