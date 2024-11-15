package frc.robot.subsystems.ShooterHood;

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

import frc.robot.Constants.ShooterHoodConstants;
import frc.robot.subsystems.ShooterHood.ShooterHoodConstants.PHConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class ShooterHoodIOPneumaticFOC implements ShooterHoodIO {
    // Hardware
    DoubleSolenoid m_shooterPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PHConstants.HoodForwardSolenoid, PHConstants.HoodReverseSolenoid);

    // Status Signals

    private Value m_pistonPosition;


    public ShooterHoodIOPneumaticFOC() {
        
    }
    
    // Update Inputs
    public void updateInputs(ShooterHoodIOInputs inputs) {
        inputs.m_pistonPosition = m_shooterPiston.get();
        
    }

    // Turn motors to stop the shooter pneumatic
    public void stop() {
        m_shooterPiston.set(Value.kOff);

    }
      //shooter up
    public void up(){
        m_shooterPiston.set(Value.kReverse);
    } 
    //shooter down  
    public void down(){
        m_shooterPiston.set(Value.kForward);
    }


}