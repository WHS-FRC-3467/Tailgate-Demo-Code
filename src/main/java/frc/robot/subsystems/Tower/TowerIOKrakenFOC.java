package frc.robot.subsystems.Tower;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.Tower.TowerConstants;

public class TowerIOKrakenFOC implements TowerIO{
    // Hardware
    private final TalonSRX m_lower;
    private final TalonSRX m_upper;

    private final DigitalInput m_entryBeamBreak = new DigitalInput(TowerConstants.DIOConstants.EntryBeamBreak);
    private final DigitalInput m_midBeamBreak = new DigitalInput(TowerConstants.DIOConstants.MidTowerBeamBreak);
    private final DigitalInput m_upperBeamBreak = new DigitalInput(TowerConstants.DIOConstants.UpperTowerBeamBreak);

    // Status Signals
    
    private final double m_lowerSupplyCurrent;
    private final double m_upperSupplyCurrent;



    // Control
    private final NeutralOut m_neutral = new NeutralOut();
    private final DutyCycleOut m_duty = new DutyCycleOut(0.0);

    public TowerIOKrakenFOC() {
        m_lower = new TalonSRX(TowerConstants.LOWER_TOWER_MOTOR);
        m_upper = new TalonSRX(TowerConstants.UPPER_TOWER_MOTOR);

        m_lower.configPeakCurrentLimit(80, 10); // in (amps, milliseconds to give it time to configure before reporting error)
        m_upper.configPeakCurrentLimit(80, 10);

        TalonFXConfiguration m_configuration = new TalonFXConfiguration();

        SupplyCurrentLimitConfiguration m_config = new SupplyCurrentLimitConfiguration(true, 60, 80, 0.1); // time threshold in seconds

        // Apply Configs
        m_lower.configSupplyCurrentLimit(m_config, 100);
        m_upper.configSupplyCurrentLimit(m_config, 100);


        // Set Signals
        m_lowerSupplyCurrent = m_lower.getSupplyCurrent();
        m_upperSupplyCurrent = m_upper.getSupplyCurrent();

        m_lower.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
        m_lower.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 255);
        m_lower.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 255);
        m_lower.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 255);
        m_lower.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 255);
        m_lower.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
    
        m_upper.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
        m_upper.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 255);
        m_upper.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 255);
        m_upper.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 255);
        m_upper.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 255);
        m_upper.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);

    }
    // Update Inputs
    public void updateInputs(ElevatorRollersIOInputs inputs) {
        inputs.lowerSupplyCurrent = m_lowerSupplyCurrent;
        inputs.upperSupplyCurrent = m_upperSupplyCurrent;
        inputs.lowBeamBreak = m_entryBeamBreak.get();
        inputs.midBeamBreak = m_midBeamBreak.get();
        inputs.highBeamBreak = m_upperBeamBreak.get();
    }

    // Turn motors to Nuetral Mode
    public void stop() {
        m_lower.set(ControlMode.Disabled, 0);
        m_upper.set(ControlMode.Disabled, 0);
    }

    // Run Duty Cycle
    public void runPercentOutput(double lowerOutput, double upperOutput) {
        // TODO: Check to see if motors run in the correct direction
        m_lower.set(ControlMode.PercentOutput, lowerOutput);
        m_upper.set(ControlMode.PercentOutput, upperOutput);
    }

}
