package frc.robot.subsystems.Tower;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElevatorRollersConstants;
import frc.robot.subsystems.Tower.TowerConstants;

public class TowerIOKrakenFOC implements TowerIO{
    // Hardware
    private final TalonFX m_lower;
    private final TalonFX m_upper;

    private final DigitalInput m_entryBeamBreak = new DigitalInput(TowerConstants.DIOConstants.EntryBeamBreak);
    private final DigitalInput m_midBeamBreak = new DigitalInput(TowerConstants.DIOConstants.MidTowerBeamBreak);
    private final DigitalInput m_upperBeamBreak = new DigitalInput(TowerConstants.DIOConstants.UpperTowerBeamBreak);

    // Status Signals
    
    private final StatusSignal<Double> m_lowerAppliedVolts;
    private final StatusSignal<Double> m_lowerSupplyCurrent;
    private final StatusSignal<Double> m_lowerTorqueCurrent;
    private final StatusSignal<Double> m_upperAppliedVolts;
    private final StatusSignal<Double> m_upperSupplyCurrent;
    private final StatusSignal<Double> m_upperTorqueCurrent;



    // Control
    private final NeutralOut m_neutral = new NeutralOut();
    private final DutyCycleOut m_duty = new DutyCycleOut(0.0);

    public TowerIOKrakenFOC() {
        m_lower = new TalonFX(ElevatorRollersConstants.ID_Motor);
        m_upper = new TalonFX(ElevatorRollersConstants.ID_Motor);

        TalonFXConfiguration m_configuration = new TalonFXConfiguration();

        m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_configuration.Voltage.PeakForwardVoltage = 12.0;
        m_configuration.Voltage.PeakReverseVoltage = -12.0;

        m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
        m_configuration.CurrentLimits.SupplyCurrentThreshold = 40;
        m_configuration.CurrentLimits.SupplyTimeThreshold = 0.1;
        m_configuration.CurrentLimits.SupplyCurrentLimitEnable = false;
        m_configuration.CurrentLimits.StatorCurrentLimit = 70;
        m_configuration.CurrentLimits.StatorCurrentLimitEnable = false;

        // Apply Configs
        m_lower.getConfigurator().apply(m_configuration);        
        m_upper.getConfigurator().apply(m_configuration);


        // Set Signals
        m_lowerAppliedVolts = m_lower.getMotorVoltage();
        m_lowerSupplyCurrent = m_lower.getSupplyCurrent();
        m_lowerTorqueCurrent = m_lower.getTorqueCurrent();

        m_upperAppliedVolts = m_upper.getMotorVoltage();
        m_upperSupplyCurrent = m_upper.getSupplyCurrent();
        m_upperTorqueCurrent = m_upper.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            m_lowerAppliedVolts,
            m_lowerSupplyCurrent,
            m_lowerTorqueCurrent,
            m_upperAppliedVolts,
            m_upperSupplyCurrent,
            m_upperTorqueCurrent
            );
    }
    // Update Inputs
    public void updateInputs(ElevatorRollersIOInputs inputs) {
        inputs.lowerMotorVoltage = m_lowerAppliedVolts.getValueAsDouble();
        inputs.lowerSupplyCurrent = m_lowerSupplyCurrent.getValueAsDouble();
        inputs.upperMotorVoltage = m_upperAppliedVolts.getValueAsDouble();
        inputs.upperSupplyCurrent = m_upperSupplyCurrent.getValueAsDouble();
        inputs.lowBeamBreak = m_entryBeamBreak.get();
        inputs.midBeamBreak = m_midBeamBreak.get();
        inputs.highBeamBreak = m_upperBeamBreak.get();
    }

    // Turn motors to Nuetral Mode
    public void stop() {
        m_lower.setControl(m_neutral);
        m_upper.setControl(m_neutral);
    }

    // Run Duty Cycle
    public void runDutyCycle(double lowerOutput, double upperOutput) {
        m_lower.setControl(m_duty.withOutput(lowerOutput));
        m_upper.setControl(m_duty.withOutput(upperOutput));
    }

}
