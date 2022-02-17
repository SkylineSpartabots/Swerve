/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package main.java.frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.LazyTalonFX;
import frc.lib.drivers.MotorChecker;
import frc.lib.drivers.PheonixUtil;
import frc.lib.drivers.TalonFXChecker;
import frc.lib.drivers.TalonFXFactory;
import frc.lib.drivers.TalonFXUtil;
import frc.lib.util.Util;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.subsystems.requests.Request;


public class IntakeSubsystem extends SubsystemBase{
    //Intake Subystem get/set Instance
    private static IntakeSubsystem instance = null;
    public static IntakeSubsystem getInstance() {
        return instance;
    }
    public static void setInstance(){
        if(instance == null){
            instance = new IntakeSubsystem();
        }
    }

    //Hardware, Soelnoids and Motors (Talon FXs)
    private final Solenoid solenoid;
    private final LazyTalonFX sTalon, mTalon;

    //Control States for Intake and Timestamp
    private static IntakeControlState CurrentState = IntakeControlState.OFF;
    private boolean stateChanged = false;
    private double stateChangeTimestamp = 0;

    //external subsystem influence
    private final Drive mDrive = Drive.getInstance();


    private void configureIntakeMotor(LazyTalonFX talon, InvertType inversion) {
        talon.setInverted(inversion);
        PheonixUtil.checkError(talon.configVoltageCompSaturation(12.0, Constants.kTimeOutMs),
            talon.getName() + " failed to set voltage compensation", true);
        PheonixUtil.checkError(talon.configVoltageMeasurementFilter(32.0, Constants.kTimeOutMs),
            talon.getName() + " failed to set voltage meas. filter", true);
        talon.enableVoltageCompensation(true);

        TalonFXUtil.setCurrentLimit(talon, 25);
        talon.setNeutralMode(NeutralMode.Coast);
    }

    //Ports for Solenoid and TalonFX motors, needs to be updated in Ports.java
    private Intake() {
        solenoid= new Solenoid(Ports.PCM_ID, Ports.INTAKE_SOLENOID_PORT);
        
        mTalon = TalonSRXFactory.createDefaultTalon("Inner Intake Motor", Ports.INTAKE_MTALON_MOTOR_ID);
        configureIntakeMotor(mTalon, InvertType.None);

        sTalon = TalonSRXFactory.createDefaultTalon("Outer Intake Motor", Ports.INTAKE_STALON_MOTOR_ID);
        configureIntakeMotor(sTalon, InvertType.InvertMotorOutput);      
    }

    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {
                synchronized(Intake.this) {
                    if(mCurrentState == IntakeControlState.STORE) {
                        double driveVelocity = mDrive.getAverageDriveVelocityMagnitude();
                        setInnerIntakeSpeed(0.0);
                        setOuterIntakeSpeed(-Util.limit(driveVelocity * 0.7, mCurrentState.outerIntakeSpeed));
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                deactivateIntake();

            }
        });
    }

    public enum IntakeControlState{
        ON(0.00, true),
        HARD_INTAKE(1.00, true),
        STORE(0.20, true),
        HARD_STORE(0.50,true),
        INTAKE(0.40, true),
        OUTTAKE(-0.40, true),
        HARD_OUTTAKE(-0.50,true),
        REVERSE(-1.0, true),
        OFF(0.00, false);

        public double intakeSpeed = 0.00;
        public boolean intakeDeployed = false;

        private IntakeControlState(double intakeSpeed, boolean intakeDeployed) {
            this.intakeSpeed = intakeSpeed;
            this.intakeDeployed = intakeDeployed;
        }
    }

    public synchronized IntakeControlState getSetState(IntakeControlState newState) {
        if(newState != CurrentState) {
            stateChanged = true;
            stateChangeTimestamp = Timer.getFPGATimestamp();
        }
        CurrentState = newState;
    }

    public synchronized void conformToState(IntakeControlState desiredState) {
        double intakeSpeed = desiredState.intakeSpeed;
        boolean intakeDeployed = desiredState.intakeDeployed;
        mTalon.set(ControlMode.PercentOutput, intakeSpeed);
        sTalon.set(ControlMode.PercentOutput, intakeSpeed);
        solenoid.set(intakeDeployed);
    }

    @Override
    public void deactivateIntake() {
        conformToState(IntakeControlState.OFF);
    }
    
    //Needs to be updated to include
    @Override
    public void autonomousInit(){

    }
    @Override
    public void autonomousPeriodic(){

    }
    @Override
    public void teleopPeriodic(){

    }

    //Debugging
    public static boolean getDebugIntake(boolean debugIntake){
        if(debugIntake == true){
            return true;
        }
        else{
            return false;
        }
    }

    public void outputTelemetry() {
        if(getDebugIntake){
            //States
            SmartDashboard.putNumber("Intake Speed", currentState.intakeSpeed);
            SmartDasboard.putBoolean("Intake On", currentState.intakeDeployed);
            //mTalon
            SmartDashboard.putNumber("Inner Intake Supply Current", mTalon.getSupplyCurrent());
            SmartDashboard.putNumber("Inner Intake Stator Current", mTalon.getStatorCurrent());
            SmartDashboard.putNumber("Inner Intake Output", mTalon.getLastSet());
            //sTalon
            SmartDashboard.putNumber("Outer Intake Supply Current", sTalon.getSupplyCurrent());
            SmartDashboard.putNumber("Outer Intake Stator Current", sTalon.getStatorCurrent());
            SmartDashboard.putNumber("Outer Intake Output", sTalon.getLastSet());
            //Solenoid
            SmartDashboard.putBoolean("Intake Solenoid", solenoid.get());
        }
    }
}
