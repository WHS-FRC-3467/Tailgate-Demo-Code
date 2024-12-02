package frc.robot.subsystems.Tower;
import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public interface TowerIO {
        // MJW: IO Layering 11/11/2024
    @AutoLog
    class TowerIOInputs { // no idea how to fix this error of the class failing to write
        // USED CURRENTLY
        double supplyCurrent = 0.0;
        double motorVoltage = 0.0;
    }

    // Update Inputs
    default void updateInputs(TowerIOInputs inputs) {}

    // Turn motors to Nuetral Mode
    default void stop() {}

    // Run Duty Cycle
    default void runDutyCycle(double output) {}
}
