// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeJoint;



import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.IntakeJointConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import frc.robot.subsystems.IntakeJoint.IntakeJointConstants;
import frc.robot.subsystems.IntakeJoint.IntakeJointIO;
import frc.robot.subsystems.IntakeJoint.IntakeJointIOInputsAutoLogged;

public class IntakeJoint extends SubsystemBase {

    @RequiredArgsConstructor
    @Getter
    public enum State {
        RETRACTED,
        EXTENDED,
        OFF;

    }

    @Getter
    @Setter
    private State state = State.RETRACTED;
    private final SendableChooser<State> stateChooser = new SendableChooser<>();
    
    //AdvantageKit addition MJW 11/11/2024
    private final IntakeJointIO io;
    private final IntakeJointIOInputsAutoLogged inputs = new IntakeJointIOInputsAutoLogged();


    /** Creates a new ComplexSubsystem. */
    public IntakeJoint(IntakeJointIO io) {
        this.io = io;

        for (State states : State.values()) {
                stateChooser.addOption(states.toString(), states);  
        }
        stateChooser.setDefaultOption(state.toString(), state);
        SmartDashboard.putData("IntakeJoint State Chooser", stateChooser);
        SmartDashboard.putData("IntakeJoint Override Command",Commands.runOnce(() -> setState(stateChooser.getSelected()), this));

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IntakeJoint", inputs);

        if (state == State.RETRACTED) {
            io.retract();

        } else if (state == State.EXTENDED) {
            io.deploy();
        } else {
            io.stop();
        }

        displayInfo(true);
    }

    public boolean atGoal() {
        return ((inputs.m_pistonPosition == Value.kReverse) && (state == State.EXTENDED)) || 
                    ((inputs.m_pistonPosition == Value.kForward) && (state == State.RETRACTED)) ||
                    ((inputs.m_pistonPosition == Value.kOff) && (state == State.OFF));
    }

    public Command setStateCommand(State state) {
        return startEnd(() -> this.state = state, () -> this.state = State.RETRACTED);
    }

    //@AutoLogOutput(key = "IntakeJoint/Info")
    private void displayInfo(boolean debug) {
        if (debug) {
            SmartDashboard.putString(this.getClass().getSimpleName() + " State ", state.toString());
            SmartDashboard.putString(this.getClass().getSimpleName() + " Output ", inputs.m_pistonPosition.toString());
            // SmartDashboard.putNumber(this.getClass().getSimpleName() + " Current Draw", inputs.supplyCurrent);
            SmartDashboard.putBoolean(this.getClass().getSimpleName() + " atGoal", atGoal());
        }

    }
}
