/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.*;
import frc.robot.Constants;


public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem mInstance = null;
    private static Solenoid solenoid;
    private static LazyTalonFX leftTalon;
    private static LazyTalonFX rightTalon;
    private IntakeControlState mCurrentState = IntakeControlState.OFF;

    public static IntakeSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new IntakeSubsystem();
        }
        return mInstance;
    }




    public enum IntakeControlState {
        OFF(0.0, false),
        ON(1.0, true);
        private boolean deployIntake = false;
        private double intakeSpeed = 0.0;

        private IntakeControlState(double intakeSpeed, boolean deployIntake) {
            this.deployIntake = deployIntake;
            this.intakeSpeed = intakeSpeed;
        }

    }
}