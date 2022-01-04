// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.SwerveDriveByVoltage;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class VoltageTestCommand extends CommandBase {
  //@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDriveByVoltage m_driveTrain;

  private final Timer m_timer = new Timer();
  private final double m_voltage;
  private final double m_durationInSecond;

  public VoltageTestCommand() 
  {
    m_driveTrain = SwerveDriveByVoltage.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);

    ShuffleboardTab tab = Shuffleboard.getTab("Test");
    m_voltage = tab.add("Voltage", 0).getEntry().getDouble(0);
    m_durationInSecond = tab.add("DurationInSec", 0).getEntry().getDouble(1);

    this.withName("VoltageTest " + m_voltage + "V for " + m_durationInSecond + " sec");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_driveTrain.DriveByVoltage(0);
  }

  @Override
  public void execute() {
    m_driveTrain.DriveByVoltage(m_voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_driveTrain.DriveByVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_durationInSecond);
  }
}
