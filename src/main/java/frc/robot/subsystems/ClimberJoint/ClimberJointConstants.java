package frc.robot.subsystems.ClimberJoint;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class ClimberJointConstants {
    public static double Tolerance = 1.0;
    
    public static final int ID_LEADER = 25;
    public static final int ID_FOLLOWER = 26;

    public static final double rightClimberMotorOffset = 0.0;
    public static final double leftClimberMotorOffset = 0.0;

    public static final int IntakeForwardSolenoid = 2;
    public static final int IntakeReverseSolenoid = 3;

    public static final double tolerance = 1;

    public static TalonFXConfiguration motorConfig() {
        TalonFXConfiguration m_configuration = new TalonFXConfiguration();

        m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        m_configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 77.6;
        m_configuration.Voltage.PeakForwardVoltage = 12.0;
        m_configuration.Voltage.PeakReverseVoltage = -12.0;

        m_configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        m_configuration.Feedback.SensorToMechanismRatio = 1;

        m_configuration.Slot0.kP = 0; // output per unit of error in position (output/rotation)
        m_configuration.Slot0.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
        m_configuration.Slot0.kD = 0; // output per unit of error derivative in position (output/rps)

            m_configuration.Slot1.kG = 0; // output to overcome gravity (output)
            m_configuration.Slot1.kS = 0; // output to overcome static friction (output)
            m_configuration.Slot1.kV = 0; // output per unit of requested velocity (output/rps)
            m_configuration.Slot1.kA = 0; // unused, as there is no target acceleration
            m_configuration.Slot1.kP = 5; // output per unit of error in position (output/rotation)
            m_configuration.Slot1.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
            m_configuration.Slot1.kD = 0; // output per unit of error derivative in position (output/rps)

            m_configuration.MotionMagic.MotionMagicCruiseVelocity = 600;
            m_configuration.MotionMagic.MotionMagicAcceleration = 200;
            m_configuration.MotionMagic.MotionMagicJerk = 0;

            m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
            m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
            m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
            m_configuration.CurrentLimits.SupplyCurrentLimitEnable = false;
            m_configuration.CurrentLimits.StatorCurrentLimit = 70;
            m_configuration.CurrentLimits.StatorCurrentLimitEnable = false;

            return m_configuration;
        }
        /**
         * 
         * PID Gains may have to be adjusted based on the responsiveness of control loop.
         * kF: 1023 represents output value to Talon at 100%, 6800 represents Velocity units at 100% output
         * 
         * 	kP   kI   kD   kF   Iz   PeakOut */
    public static final Gains kGains_Distance =
        switch (Constants.currentMode) {
            case SIM -> new Gains(1, 0, 0, 0, 0, 0);
            case REAL -> new Gains(0.5, 0, 0.01, 0.05, 100, 1.00);
            default -> new Gains(1, 0, 0, 0, 0.19, 0);
        };

    public final static Gains kGains_Turning  = 
        switch (Constants.currentMode) {
            case SIM -> new Gains(0,0,0,0,0,0);
            case REAL -> new Gains( 0.2, 0.0, 0.0, 0.0, 200, 1.00 );
            default -> new Gains(1,0,0,0,0.19,0);

        };
        
	
	    /* Motor neutral dead-band : Range 0.001 -> 0.25 */
	    public final static double kNeutralDeadband = 0.001;

	    /* Current Limit for arm calibration */
        public final static double kCalibCurrentLimit = 10.0;

        /**
    	 * Set to zero to skip waiting for confirmation.
	     * Set to nonzero to wait and report to DS if action fails.
	    */
	    public final static int kTimeoutMs = 30;

        // Motion Magic constants
        public static final int kMotionCruiseVelocity = 25000;
        public static final int kMotionAcceleration = 35000;
        public static final int kSlowMotionAccel = 19000;
        public final static int kCurveSmoothing = 0;  /* Valid values: 0 -> 8 */
        public static final int kTolerance = 500;

        // Motion Magic constants
        public static final int kMotionCruiseVelocityFast = 30000;
        public static final int kMotionAccelerationFast = 70000;
        public static final int kSlowMotionAccelFast = 55000;
        
        // Setpoints (in encoder ticks) (not tuned)
        public static final double kClimbingRetractedPostion = 1000.0;
        public static final double kRestingRetractedPostion = 4000.0;
        public static final double kExtendedAboveBar = 50000.0;
        public static final double kFixedArmsFree = 70000.0;
        public static final double kFullExtendedPosition = 205000.0;

    public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
