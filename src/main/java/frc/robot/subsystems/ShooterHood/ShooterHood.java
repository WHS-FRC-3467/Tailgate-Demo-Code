package frc.robot.subsystems.ShooterHood;

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
//import frc.robot.Constants.ShooterHoodConstants;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import frc.robot.subsystems.ShooterHood.ShooterHoodConstants;
import frc.robot.subsystems.ShooterHood.ShooterHoodIO;
import frc.robot.subsystems.ShooterHood.ShooterHoodIOInputsAutoLogged;

public class ShooterHood extends SubsystemBase {

    @RequiredArgsConstructor
    @Getter
    public enum State {
        FORWARD,
        REVERSE,
        OFF;

    }

    @Getter
    @Setter
    private State state = State.OFF;
    private final SendableChooser<State> stateChooser = new SendableChooser<>();
    
    //AdvantageKit addition MJW 11/11/2024
    private final ShooterHoodIO io;
    private final ShooterHoodIOInputsAutoLogged inputs = new ShooterHoodIOInputsAutoLogged();


    /** Creates a new ComplexSubsystem. */
    public ShooterHood(ShooterHoodIO io) {
        this.io = io;

        for (State states : State.values()) {
                stateChooser.addOption(states.toString(), states);  
        }
        stateChooser.setDefaultOption(state.toString(), state);
        SmartDashboard.putData("ShooterHood State Chooser", stateChooser);
        SmartDashboard.putData("ShooterHood Override Command",Commands.runOnce(() -> setState(stateChooser.getSelected()), this));

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ShooterHood", inputs);

        if (state == State.REVERSE) {
            io.reverse();

        } else if (state == State.FORWARD) {
            io.forward();
        } else {
            io.stop();
        }

        displayInfo(true);
    }

    public boolean atGoal() {
        return ((inputs.m_pistonPosition == Value.kReverse) && (state == State.REVERSE)) || 
                    ((inputs.m_pistonPosition == Value.kForward) && (state == State.FORWARD)) ||
                    ((inputs.m_pistonPosition == Value.kOff) && (state == State.OFF));
    }

    public Command setStateCommand(State state) {
        return startEnd(() -> this.state = state, () -> this.state = State.OFF);
    }

    //@AutoLogOutput(key = "ShooterHood/Info")
    private void displayInfo(boolean debug) {
        if (debug) {
            SmartDashboard.putString(this.getClass().getSimpleName() + " State ", state.toString());
            SmartDashboard.putString(this.getClass().getSimpleName() + " Output ", inputs.m_pistonPosition.toString());
            // SmartDashboard.putNumber(this.getClass().getSimpleName() + " Current Draw", inputs.supplyCurrent);
            SmartDashboard.putBoolean(this.getClass().getSimpleName() + " atGoal", atGoal());
        }

    }
}
