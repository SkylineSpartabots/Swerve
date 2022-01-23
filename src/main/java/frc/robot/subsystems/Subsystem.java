/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

/*
 * I'm not quite sure why I need this but stuff breaks when its
 * gone, and I don't feel like breaking all the Talon libs.
 *
 * So here we are.
 */
public abstract class Subsystem {

    public boolean hasEmergency = false;

    public void writeToLog() {}

    public void readPeriodicInputs() {}

    public void writePeriodicOutputs() {}

    public void zeroSensors() {}

    public abstract void stop();

    public abstract boolean checkSystem();

    public abstract void outputTelemetry();
}
