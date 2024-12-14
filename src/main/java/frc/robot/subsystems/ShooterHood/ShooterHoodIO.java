package frc.robot.subsystems.ShooterHood;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public interface ShooterHoodIO {
    Value m_pistonPosition = null;

    // MJW: IO Layering 11/11/2024
    @AutoLog
    class ShooterHoodIOInputs {
        // USED CURRENTLY
        Value m_pistonPosition = Value.kOff;
    }

    // Update Inputs
    default void updateInputs(ShooterHoodIOInputs inputs) {}

    // makes shooter hood go forward
    default void forward() {} 
    
    // reverses shooter hood  
    default void reverse(){}

    
    default void stop(){}
}