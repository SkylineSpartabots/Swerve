package frc.robot.commands;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class DriveCommandFactory {
    private static DrivetrainSubsystem subsystem = DrivetrainSubsystem.getInstance();

    public static DriveCommand createDriveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier angle){
        return new DriveCommand(subsystem, x, y, angle);
    }
    public static DriveCommand createDriveToPositionCommand(double setpointX, double setpointY, double angle){
        PIDController controller = subsystem.getPIDController();
        Pose2d pose = subsystem.getPose();
        DoubleSupplier xSupplier = () -> setpointX / 2;
        SmartDashboard.putNumber("POWER OUTPUTTED BY AUTO", xSupplier.getAsDouble());
        DoubleSupplier ySupplier = () -> 0;//controller.calculate(pose.getTranslation().getY(), setpointY);
        DoubleSupplier angleSupplier = () ->  0;//controller.calculate(subsystem.getGyroscopeRotation().getDegrees(), angle);
        DriveCommand command = new DriveCommand(subsystem, xSupplier, ySupplier, angleSupplier);
        command.setTarget(setpointX, setpointY, angle);
        return command;
    }
}