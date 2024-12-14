// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Tower;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Tower extends SubsystemBase {

    @RequiredArgsConstructor
    @Getter
    public enum State {
        OFF,
        INTAKE,
        SHOOT;
    }

    @RequiredArgsConstructor
    @Getter
    public enum Action {
        OFF(0.0, 0.0), // doubles are lower tower motor output, upper tower motor output
        RUNTOWER(0.75, 0.75), // Intaking and If no balls are Top Position yet (BB2)
        LOWERTOWER(1.0, 0.0), // If intaking and 1 ball is in Top Position
        SHOOT(1.0, 1.0); // If shooting (only if ball is in Top Position/BB2)

        private final double lowerOutput;
        private final double upperOutput;
    }

    // Command actions
    // shuffle bals to the top of tower w/ intake
    // move tower without intake
    // move whole tower (for shooting)

    @RequiredArgsConstructor
    public enum TowerStatus {
        NOBALLS(1,0),
        LOWER(2,1),
        MIDDLE(3,1),
        UPPER(4,1),
        MIDDLEANDUPPER(5, 2),
        LOWERANDUPPER(6, 2),
        LOWERANDMIDDLE(7, 2),
        ALLBROKEN(8, 3); // All three beambreaks - either three balls or something is wrong

        private final int statusNumber;
        private final int numBalls;
    }

    @Getter
    @Setter
    private State state = State.OFF;

    @Getter
    @Setter
    private Action action = Action.OFF;

    @Getter
    @Setter
    private TowerStatus status = TowerStatus.NOBALLS;

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
        getStatus();
        if (state == State.OFF) {
            //m_motor.setControl(m_neutral);
            action = Action.OFF;
        } else if (state == State.INTAKE) {
                    //Old way: m_motor.setControl(m_percent.withOutput(action.getOutput()));
            // Intaking: Evaluate the tower's status and make it run top/bottom motors based on the tower's status
            if ((status == TowerStatus.NOBALLS) || (status == TowerStatus.UPPER) || (status == TowerStatus.MIDDLEANDUPPER)) {
                action = Action.OFF; 
                // If there are no balls in the Tower that are below the top beambreak, or if the Tower is Full, stop tower motors
            } else if ((status == TowerStatus.LOWER) || (status == TowerStatus.MIDDLE) || (status == TowerStatus.LOWERANDMIDDLE)) {
                action = Action.RUNTOWER; // If there is a ball in the tower but it isn't at the top, then run whole tower.
            } else if (status == TowerStatus.LOWERANDUPPER) {
                action = Action.LOWERTOWER; // 1 Ball in Upper, one ball in the bottom of the tower --> need to shuffle it up
            } else {
                action = Action.OFF;
            }

        } else if (state == State.SHOOT) {
            // If in mode SHOOT, evaluate whether the robot should be shooting
            if ((status == TowerStatus.MIDDLEANDUPPER) || (status == TowerStatus.UPPER)) {
                action = Action.SHOOT;
            } else {
                action = Action.OFF;
            }
        }
        // Now that the robot knows what to do, let's run the motors (or not)
        if (action == Action.OFF){
            io.stop();
        } else {
            io.runDutyCycle(action.getLowerOutput(), action.getUpperOutput());
        }

        displayInfo(debug);
    }

    public TowerStatus getStatus() {
        // Does the beambreak logic. Stores it in status for calling on smartdashboard and periodic
        if ((inputs.lowBeamBreak == false) && (inputs.midBeamBreak == false) && (inputs.highBeamBreak == false)) {
            status = TowerStatus.NOBALLS;
        } else if ((inputs.lowBeamBreak == true) && (inputs.midBeamBreak == false) && (inputs.highBeamBreak == false)) {
            status = TowerStatus.LOWER;
        } else if ((inputs.lowBeamBreak == false) && (inputs.midBeamBreak == true) && (inputs.highBeamBreak == false)) {
            status = TowerStatus.MIDDLE;
        } else if ((inputs.lowBeamBreak == false) && (inputs.midBeamBreak == false) && (inputs.highBeamBreak == true)) {
            status = TowerStatus.UPPER;
        } else if ((inputs.lowBeamBreak == false) && (inputs.midBeamBreak == true) && (inputs.highBeamBreak == true)) {
            status = TowerStatus.MIDDLEANDUPPER;
        } else if ((inputs.lowBeamBreak == true) && (inputs.midBeamBreak == false) && (inputs.highBeamBreak == true)) {
            status = TowerStatus.LOWERANDUPPER;
        } else if ((inputs.lowBeamBreak == true) && (inputs.midBeamBreak == true) && (inputs.highBeamBreak == false)) {
            status = TowerStatus.LOWERANDMIDDLE;
        } else {
            status = TowerStatus.ALLBROKEN;
        }
        return status;
    }

    public Command setStateCommand(State state) {
        return startEnd(() -> this.state = state, () -> this.state = State.OFF);
    }

    //@AutoLogOutput(key = "ElevatorRollers/Info")
    private void displayInfo(boolean debug) {
        if (debug) {
            SmartDashboard.putString(this.getClass().getSimpleName() + " state ", state.toString());
            SmartDashboard.putString(this.getClass().getSimpleName() + " action ", action.toString());
            SmartDashboard.putString(this.getClass().getSimpleName() + "  Status ", status.toString());
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Lower Setpoint ", action.getLowerOutput());   
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Upper Setpoint ", action.getUpperOutput());

            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Output Lower motor", inputs.lowerMotorVoltage);
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Current Draw Lower motor", inputs.lowerSupplyCurrent);
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Output Upper motor", inputs.upperMotorVoltage);
            SmartDashboard.putNumber(this.getClass().getSimpleName() + " Current Draw Upper motor", inputs.upperSupplyCurrent);
        }

    }

}