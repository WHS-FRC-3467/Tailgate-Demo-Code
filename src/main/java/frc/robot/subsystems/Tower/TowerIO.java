package frc.robot.subsystems.Tower;

import org.littletonrobotics.junction.AutoLog;

public interface TowerIO {
    // MJW: IO Layering 11/11/2024
    @AutoLog
    class ElevatorRollersIOInputs {
        // USED CURRENTLY
        double supplyCurrent = 0.0;
        double motorVoltage = 0.0;
    }

    // Update Inputs
    default void updateInputs(ElevatorRollersIOInputs inputs) {}

    // Turn motors to Nuetral Mode
    default void stop() {}

    // Run Duty Cycle
    default void runDutyCycle(double output) {}
}
