// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Tower;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ShooterHood.ShooterHoodIO.ShooterHoodIOInputs;
import frc.robot.subsystems.Tower.TowerConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import frc.robot.subsystems.Tower.TowerIO.TowerIOInputs;

public class Tower extends SubsystemBase {

    @RequiredArgsConstructor
    @Getter
    public enum State {
        OFF(0.0),
        INTAKE(0.25),
        SCORE(0.75),
        EJECT(-0.30);

        private final double output;
    }

    @RequiredArgsConstructor
    @Getter
    //from Noah
    public enum TowerStatus {
        NOBALLS(1,0),
        LOWER(2,1),
        MIDDLE(3,1),
        UPPER(4,1),
        MIDDLEANDUPPER(5, 2),
        LOWERANDUPPER(6, 2),
        LOWERANDMIDDLE(7, 2),   
        ALLBROKEN(8, 3); 
        

        private final int statusNumber;
        private final int numBalls;
    }



    @Getter
    @Setter
    private State state = State.OFF;

    private boolean debug = true;

    //TalonFX m_motor = new TalonFX(Tower.ID_Motor);
    //private final DutyCycleOut m_percent = new DutyCycleOut(0);
    //private final NeutralOut m_neutral = new NeutralOut();

    //AdvantageKit addition MJW 11/11/2024
    private final TowerIO io;
    private final TowerIOInputs inputs = new TowerIOInputs();

    /** Creates a new SimpleSubsystem. */
    public Tower(TowerIO io) {
        this.io = io;
        //m_motor.getConfigurator().apply(Tower.motorConfig());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Tower", (LoggableInputs) inputs);
        if (state == State.OFF) {
            //m_motor.setControl(m_neutral);
            io.stop();
        } else {
            //m_motor.setControl(m_percent.withOutput(state.getOutput()));
            io.runDutyCycle(state.getOutput());
        }

        displayInfo(debug);
    }

    public Command setStateCommand(State state) {
        return startEnd(() -> this.state = state, () -> this.state = State.OFF);
    }

    private void displayInfo(boolean debug) {
        if (debug) {
            SmartDashboard.putString(this.getClass().getSimpleName() + " State ", state.toString());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Setpoint ", state.getOutput());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Output ", inputs.motorVoltage);
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Current Draw", inputs.supplyCurrent);
        }

    }
}