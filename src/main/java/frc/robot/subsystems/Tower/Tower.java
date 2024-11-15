// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Tower;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorRollersConstants;
import frc.robot.subsystems.Tower.ElevatorRollersIOInputsAutoLogged;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Tower extends SubsystemBase {

    @RequiredArgsConstructor
    @Getter
    public enum State {
        OFF(0.0),
        SHUFFLEWITHINTAKE(0.0),
        SHUFFLENOINTAKE(0.75),
        SHOOT(-0.30);

        private final double output;
    }

    // Command states
    // shuffle bals to the top of tower w/ intake
    // move tower without intake
    // move whole tower (for shooting)

    @RequiredArgsConstructor
    @Getter
    public enum TowerStatus {
        NOBALLS(1,0),
        ENTRY(2,1),
        MIDDLE(3,1),
        UPPER(4,1),
        MIDDLEANDUPPER(5, 2),
        ENTRYANDUPPER(6, 2),
        ENTRYANDMIDDLE(7, 2),
        ALLBROKEN(8, 3); // All three beambreaks - either three balls or something is wrong

        private final int statusNumber;
        private final int numBalls;
    }

    @Getter
    @Setter
    private State state = State.OFF;

    private boolean debug = true;

    //TalonFX m_motor = new TalonFX(ElevatorRollersConstants.ID_Motor);
    //private final DutyCycleOut m_percent = new DutyCycleOut(0);
    //private final NeutralOut m_neutral = new NeutralOut();

    //AdvantageKit addition MJW 11/11/2024
    private final TowerIO io;
    private final ElevatorRollersIOInputsAutoLogged inputs = new ElevatorRollersIOInputsAutoLogged();

    /** Creates a new SimpleSubsystem. */
    public Tower(TowerIO io) {
        this.io = io;
        //m_motor.getConfigurator().apply(ElevatorRollersConstants.motorConfig());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ElevatorRollers", inputs);
        if (state == State.OFF) {
            //m_motor.setControl(m_neutral);
            io.stop();
        } else if (state == State.SHUFFLENOINTAKE) {
            // TODO: Evaluate the tower's status and make it run top/bottom motors based on the tower's status
            // Make that method in TowerIOKrakenFOC.java
        } else if (state == State.SHUFFLEWITHINTAKE) {
            // TODO: Evaluate the tower's status and make it run top/bottom motors based on the tower's status

        } else {
            // Shoot! Full Tower Goes!
            //m_motor.setControl(m_percent.withOutput(state.getOutput()));
            io.runDutyCycle(state.getOutput());
        }

        displayInfo(debug);
    }

    public TowerStatus getStatus() {
        // TODO: DO The beambreak logic
        return TowerStatus.NOBALLS;
    }

    public Command setStateCommand(State state) {
        return startEnd(() -> this.state = state, () -> this.state = State.OFF);
    }

    //@AutoLogOutput(key = "ElevatorRollers/Info")
    private void displayInfo(boolean debug) {
        if (debug) {
            SmartDashboard.putString(this.getClass().getSimpleName() + " State ", state.toString());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Setpoint ", state.getOutput());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Output Lower motor", inputs.lowerMotorVoltage);
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Current Draw Lower motor", inputs.lowerSupplyCurrent);
        }

    }

    public int ballCount() {
        return 0;
    }
}