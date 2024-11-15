package frc.robot.subsystems.IntakeJoint;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.util.Units;

import frc.robot.Constants.IntakeJointConstants;
import frc.robot.subsystems.IntakeJoint.IntakeJointConstants.PHConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class IntakeJointIOPneumaticFOC implements IntakeJointIO {
    // Hardware
    DoubleSolenoid m_intakePiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PHConstants.IntakeForwardSolenoid, PHConstants.IntakeReverseSolenoid);

    // Status Signals

    private Value m_pistonPosition;


    public IntakeJointIOPneumaticFOC() {


        
    }
    // Update Inputs
    public void updateInputs(IntakeJointIOInputs inputs) {
        inputs.m_pistonPosition = m_intakePiston.get();
        
    }

    // Turn motors to stop the intake pneumatic
    public void stop() {
        m_intakePiston.set(Value.kOff);

    }
      //deploys intake
    public void deploy(){
        m_intakePiston.set(Value.kReverse);
    } 
    //retracts intake  
    public void retract(){
        m_intakePiston.set(Value.kForward);
    }


}