package frc.robot.subsystems.IntakeJoint;

import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public interface IntakeJointIO {
    Value m_pistonPosition = null;

    // MJW: IO Layering 11/11/2024
    @AutoLog
    class IntakeJointIOInputs {
        // USED CURRENTLY
        Value m_pistonPosition = Value.kOff;
    }

    // Update Inputs
    default void updateInputs(IntakeJointIOInputs inputs) {}

    // Intake down
    default void deploy() {} 
    
    // retracts intake  
    default void retract(){}

    // retracts intake  
    default void stop(){}
}
