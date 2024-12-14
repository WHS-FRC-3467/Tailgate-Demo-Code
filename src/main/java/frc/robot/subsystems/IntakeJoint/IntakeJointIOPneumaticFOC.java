package frc.robot.subsystems.IntakeJoint;

import com.google.flatbuffers.Constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class IntakeJointIOPneumaticFOC implements IntakeJointIO {
    // Hardware
    DoubleSolenoid m_intakePiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, IntakeJointConstants.IntakeForwardSolenoid, IntakeJointConstants.IntakeReverseSolenoid);

        // solenoid value that will be updated in periodic
    private Value m_pistonPosition;

    public IntakeJointIOPneumaticFOC() {

    }

    // Update Inputs
    public void updateInputs(IntakeJointIOInputs inputs) {
        inputs.m_pistonPosition = m_intakePiston.get();
        
    }

    // Turn motors to stop the intake pneumatic
    public void stop() {
        m_intakePiston.set(Value.kOff);
    }

      //deploys intake
    public void deploy(){
        m_intakePiston.set(Value.kReverse);
    } 

    //retracts intake  
    public void retract(){
        m_intakePiston.set(Value.kForward);
    }

}