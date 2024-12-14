package frc.robot.subsystems.ShooterRollers;

import frc.robot.Constants;

public class ShooterRollersConstants {

    double Tolerance = 5.00;
    public static final Gains gains =
        switch (Constants.currentMode) {
            case SIM -> new Gains(0, 0, 0, 0, 0, 0);
            case REAL -> new Gains(0.1, 0.0001, 5, 0.0495, 300, 1.0); // Gains copied from Tailgate repo
            default -> new Gains(0.1, 0.0001, 5, 0.0495, 300, 1.0); // Gains copied from Tailgate repo
        };

    public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
