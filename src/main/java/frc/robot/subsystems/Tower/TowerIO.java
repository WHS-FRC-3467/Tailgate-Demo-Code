package frc.robot.subsystems.Tower;

import org.littletonrobotics.junction.AutoLog;

public interface TowerIO {
    // MJW: IO Layering 11/11/2024
    @AutoLog
    class ElevatorRollersIOInputs {
        // USED CURRENTLY
        double lowerSupplyCurrent = 0.0;
        double lowerMotorVoltage = 0.0;
        double upperSupplyCurrent = 0.0;
        double upperMotorVoltage = 0.0;
        
        // Beambreak booleans
        boolean m_entryBeam;
        boolean m_midBeam;
        boolean m_upperBeam;
    }

    // Update Inputs
    default void updateInputs(ElevatorRollersIOInputs inputs) {}

    // Turn motors to Nuetral Mode
    default void stop() {}

    // Run Duty Cycle
    default void runDutyCycle(double output) {}
}
