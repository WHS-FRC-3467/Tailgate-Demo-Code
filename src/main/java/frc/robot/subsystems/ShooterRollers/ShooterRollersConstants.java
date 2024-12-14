package frc.robot.subsystems.ShooterRollers;

import frc.robot.Constants;

public class ShooterRollersConstants {

    public static final int ID_LEADER = 15;
    public static final int ID_FOLLOWER = 16;

    //RPS
    public static final double upperLimit = 50.0;
    public static final double lowerLimit = -upperLimit;
    public static final double tolerance = 75/60;
    public static final double kAccelCompFactor = 0.100; // in units of seconds
    public static final Gains gains =
        switch (Constants.currentMode) {
            case SIM -> new Gains(0, 0, 0, 0, 0, 0);
            case REAL -> new Gains(0.1, 0.0001, 5, 0.0495, 0, 0); // Gains copied from Tailgate repo
            default -> new Gains(0.1, 0.0001, 5, 0.0495, 0, 0); // Gains copied from Tailgate repo
        };

    public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
