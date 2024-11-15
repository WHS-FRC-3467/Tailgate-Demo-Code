package frc.robot.subsystems.ShooterHood;

import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public interface ShooterHoodIO {
    // MJW: IO Layering 11/11/2024
    @AutoLog
    class ShooterHoodIOInputs {
        double position = 0.0;
        double positionDegrees = 0.0;
        double supplyCurrent = 0.0;
        double absolutePosition = 0.0;
        double absolutePositionDegrees = 0.0;
        double motorVelocity = 0.0;
        Value m_pistonPosition = Value.kOff;
    }

    // Update Inputs
    default void updateInputs(ShooterHoodIOInputs inputs) {}

    // Turn motors to Nuetral Mode
    default void stop() {}

    // Run Duty Cycle
    default void runDutyCycle(double output) {}

    // Set MotionMagic Control
    default void setControl(MotionMagicVoltage goal) {}

    // Set Setpoint
    default void setPosition(PositionVoltage positionVoltage) {}
}
