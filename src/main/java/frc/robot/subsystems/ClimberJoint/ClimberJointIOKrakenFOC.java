package frc.robot.subsystems.ClimberJoint;

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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;


public class ClimberJointIOKrakenFOC implements ClimberJointIO {
    // Hardware
    private final TalonFX m_rightLeader;
    private final TalonFX m_leftFollower;

    DoubleSolenoid m_intakePiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ClimberJointConstants.IntakeForwardSolenoid, ClimberJointConstants.IntakeReverseSolenoid);

        // solenoid value that will be updated in periodic
    private Value m_pistonPosition;

    // Status Signals

    private final StatusSignal<Double> m_rightLeaderPosition;
    private final StatusSignal<Double> m_rightLeaderVelocity;
    private final StatusSignal<Double> m_rightLeaderAppliedVolts;
    private final StatusSignal<Double> m_rightLeaderSupplyCurrent;
    private final StatusSignal<Double> m_rightLeaderTorqueCurrent;

    private final StatusSignal<Double> m_leftFollowerPosition;
    private final StatusSignal<Double> m_leftFollowerVelocity;
    private final StatusSignal<Double> m_leftFollowerAppliedVolts;
    private final StatusSignal<Double> m_leftFollowerSupplyCurrent;
    private final StatusSignal<Double> m_leftFollowerTorqueCurrent;

    // Control
    private final VelocityVoltage m_velocity = new VelocityVoltage(0).withSlot(1); //Line 50
    private final NeutralOut m_neutral = new NeutralOut().withUpdateFreqHz(0.0); //Line 51
    private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
    private final DutyCycleOut m_duty = new DutyCycleOut(0.0);

    public ClimberJointIOKrakenFOC() {
        m_rightLeader = new TalonFX(ClimberJointConstants.ID_LEADER);
        m_leftFollower = new TalonFX(ClimberJointConstants.ID_FOLLOWER);
        // Climber Winch Motors
        TwinTalonFXMech m_talonMech = new TwinTalonFXMech(ClimberJointConstants.ID_LEADER, ClimberJointConstants.ID_FOLLOWER);


        TalonFXConfiguration m_configuration = new TalonFXConfiguration();

        m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
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

        // Apply Configs
        m_rightLeader.getConfigurator().apply(m_configuration);
        m_leftFollower.getConfigurator().apply(m_configuration);
        m_leftFollower.setControl(new Follower(m_rightLeader.getDeviceID(), true));

        // Set Signals
        m_rightLeaderPosition = m_rightLeader.getPosition();
        m_rightLeaderVelocity = m_rightLeader.getVelocity();
        m_rightLeaderAppliedVolts = m_rightLeader.getMotorVoltage();
        m_rightLeaderSupplyCurrent = m_rightLeader.getSupplyCurrent();
        m_rightLeaderTorqueCurrent = m_rightLeader.getTorqueCurrent();

        m_leftFollowerPosition = m_leftFollower.getPosition();
        m_leftFollowerVelocity = m_leftFollower.getVelocity();
        m_leftFollowerAppliedVolts = m_leftFollower.getMotorVoltage();
        m_leftFollowerSupplyCurrent = m_leftFollower.getSupplyCurrent();
        m_leftFollowerTorqueCurrent = m_leftFollower.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            m_rightLeaderPosition,
            m_rightLeaderVelocity,
            m_rightLeaderAppliedVolts,
            m_rightLeaderSupplyCurrent,
            m_rightLeaderTorqueCurrent,
            m_leftFollowerPosition,
            m_leftFollowerVelocity,
            m_leftFollowerAppliedVolts,
            m_leftFollowerSupplyCurrent,
            m_leftFollowerTorqueCurrent
            );
    }

    // Update Inputs
    public void updateInputs(ClimberJointIOInputs inputs) {
        inputs.motorVelocity = m_rightLeaderVelocity.getValueAsDouble();
        inputs.position = m_rightLeaderPosition.getValueAsDouble();
        inputs.supplyCurrent = m_rightLeaderSupplyCurrent.getValueAsDouble();
    }

    // Turn motors to Nuetral Mode
    public void stop() {
        m_rightLeader.setControl(m_neutral);
        m_leftFollower.setControl(m_neutral);
    }

    // Run Duty Cycle
    public void runDutyCycle(double output) {
        m_rightLeader.setControl(m_duty.withOutput(output));
    }

    // Move to setpoint
    public void setControl(MotionMagicVoltage goal) {
        m_rightLeader.setControl(goal.withSlot(1));
    }

    public void setPosition(double goal) {
        m_rightLeader.setPosition(goal);
    }

    public boolean calibrate(boolean left) {

		boolean isFinished = false;
		double current = 0.0;
		try (TalonFX talon = (left ? m_leftFollower : m_rightLeader)) {
            current = talon.getStatorCurrent().getValueAsDouble();
            // SmartDashboard.putNumber("Calib Curr " + (left ? "L: " : "R: "), current);
            if (current < ClimberJointConstants.kCalibCurrentLimit) {
            	talon.set(-0.20);
            	isFinished = false;
            } else {
            	talon.set(0.0);
            	isFinished = true;
            }
        }

		return isFinished;
	}

}
