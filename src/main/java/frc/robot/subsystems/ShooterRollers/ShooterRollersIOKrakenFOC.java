package frc.robot.subsystems.ShooterRollers;

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

public class ShooterRollersIOKrakenFOC implements ShooterRollersIO {
    // Hardware
    private final TalonFX RightTalon;
    private final TalonFX LeftTalon;

    // Status Signals
    
    private final StatusSignal<Double> RightPosition;
    private final StatusSignal<Double> RightVelocity;
    private final StatusSignal<Double> RightAppliedVolts;
    private final StatusSignal<Double> RightSupplyCurrent;
    private final StatusSignal<Double> RightTorqueCurrent;
    private final StatusSignal<Double> LeftPosition;
    private final StatusSignal<Double> LeftVelocity;
    private final StatusSignal<Double> LeftAppliedVolts;
    private final StatusSignal<Double> LeftSupplyCurrent;
    private final StatusSignal<Double> LeftTorqueCurrent;
    
    // Control
    private final VelocityVoltage m_velocity = new VelocityVoltage(0).withSlot(1); //Line 50
    private final NeutralOut m_neutral = new NeutralOut().withUpdateFreqHz(0.0); //Line 51
    private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);

    public ShooterRollersIOKrakenFOC() {
        LeftTalon = new TalonFX(ShooterRollersConstants.ID_LEADER); // Leader
        RightTalon = new TalonFX(ShooterRollersConstants.ID_FOLLOWER); // Follower
        // General Config
        TalonFXConfiguration m_configuration = new TalonFXConfiguration();
        m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_configuration.Voltage.PeakForwardVoltage = 12.0;
        m_configuration.Voltage.PeakReverseVoltage = -12.0;

        m_configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        m_configuration.Slot0.kP = 1; // output per unit of error in position (output/rotation)
        m_configuration.Slot0.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
        m_configuration.Slot0.kD = 0; // output per unit of error derivative in position (output/rps)

        m_configuration.Slot1.kG = 0; // output to overcome gravity (output)
        m_configuration.Slot1.kS = 0.0495; // output to overcome static friction (output)
        m_configuration.Slot1.kV = 0.13; // output per unit of requested velocity (output/rps)
        m_configuration.Slot1.kA = 0; // unused, as there is no target acceleration
        m_configuration.Slot1.kP = 0.1; // output per unit of error in position (output/rotation)
        m_configuration.Slot1.kI = 0.0001; // output per unit of integrated error in position (output/(rotation*s))
        m_configuration.Slot1.kD = 5; // output per unit of error derivative in position (output/rps)

        /* m_configuration.MotionMagic.MotionMagicCruiseVelocity = 10;
        m_configuration.MotionMagic.MotionMagicAcceleration = 10;
        m_configuration.MotionMagic.MotionMagicJerk = 10; */

        m_configuration.CurrentLimits.SupplyCurrentLimit = 60;
        m_configuration.CurrentLimits.SupplyCurrentThreshold = 80;
        m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
        m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_configuration.CurrentLimits.StatorCurrentLimit = 70;
        m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;

        // Apply Configs
        LeftTalon.getConfigurator().apply(m_configuration);
                // m_configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        RightTalon.getConfigurator().apply(m_configuration);
        RightTalon.setControl(new Follower(LeftTalon.getDeviceID(), true));

        // Set Signals
        LeftPosition = LeftTalon.getPosition();
        LeftVelocity = LeftTalon.getVelocity();
        LeftAppliedVolts = LeftTalon.getMotorVoltage();
        LeftSupplyCurrent = LeftTalon.getSupplyCurrent();
        LeftTorqueCurrent = LeftTalon.getTorqueCurrent();

        RightPosition = RightTalon.getPosition();
        RightVelocity = RightTalon.getVelocity();
        RightAppliedVolts = RightTalon.getMotorVoltage();
        RightSupplyCurrent = RightTalon.getSupplyCurrent();
        RightTorqueCurrent = RightTalon.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            LeftPosition,
            LeftVelocity,
            LeftAppliedVolts,
            LeftSupplyCurrent,
            LeftTorqueCurrent,
            RightPosition,
            RightVelocity,
            RightAppliedVolts,
            RightSupplyCurrent,
            RightTorqueCurrent);
    }

    @Override
    public void updateInputs(ShooterRollersIOInputs inputs) {
        inputs.RightPositionRads = Units.rotationsToRadians(RightPosition.getValueAsDouble());
        inputs.RightVelocityRpm = RightVelocity.getValueAsDouble() * 60.0;
        inputs.RightAppliedVolts = RightAppliedVolts.getValueAsDouble();
        inputs.RightSupplyCurrentAmps = RightSupplyCurrent.getValueAsDouble();
        inputs.RightTorqueCurrentAmps = RightTorqueCurrent.getValueAsDouble();

        inputs.LeftPositionRads = Units.rotationsToRadians(LeftPosition.getValueAsDouble());
        inputs.LeftVelocityRpm = LeftVelocity.getValueAsDouble() * 60.0;
        inputs.LeftAppliedVolts = LeftAppliedVolts.getValueAsDouble();
        inputs.LeftSupplyCurrentAmps = LeftSupplyCurrent.getValueAsDouble();
        inputs.LeftTorqueCurrentAmps = LeftTorqueCurrent.getValueAsDouble();

        // CURRENTLY USED
        inputs.motorVelocity = RightTalon.getVelocity().getValueAsDouble();
    }

    @Override
    public void runVolts(double RightVolts, double LeftVolts) {
        LeftTalon.setControl(voltageControl.withOutput(LeftVolts));
    }

    @Override
    public void stop() {
        LeftTalon.setControl(m_neutral);
    }

    @Override
    public void runVelocity(double Rpm, double Feedforward) {
        //LeftTalon.setControl(m_velocity.withVelocity(Rpm/60.0).withFeedForward(Feedforward));
        LeftTalon.set(Rpm/2500);
    }

    @Override
    public void setPoint(double goalSpeed) {
        LeftTalon.setControl(m_velocity.withVelocity(goalSpeed).withSlot(1)); // create a velocity closed-loop request, voltage output, slot 1 configs
    }
}
