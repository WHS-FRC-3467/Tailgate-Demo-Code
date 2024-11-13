// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;

import org.opencv.photo.Photo;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SensorConstants;
import frc.robot.RobotState.TARGET;
import frc.robot.Util.LaserCanSensor;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ClimberJoint.ClimberJoint;
import frc.robot.subsystems.ClimberJoint.ClimberJointIO;
import frc.robot.subsystems.ClimberJoint.ClimberJointIOKrakenFOC;
import frc.robot.subsystems.ClimberJoint.ClimberJointIOSim;
import frc.robot.subsystems.ElevatorRollers.ElevatorRollers;
import frc.robot.subsystems.ElevatorRollers.ElevatorRollersIO;
import frc.robot.subsystems.ElevatorRollers.ElevatorRollersIOKrakenFOC;
import frc.robot.subsystems.ElevatorRollers.ElevatorRollersIOSim;
import frc.robot.subsystems.IntakeJoint.IntakeJoint;
import frc.robot.subsystems.IntakeJoint.IntakeJointIO;
import frc.robot.subsystems.IntakeJoint.IntakeJointIOKrakenFOC;
import frc.robot.subsystems.IntakeJoint.IntakeJointIOSim;
import frc.robot.subsystems.IntakeRollers.IntakeRollers;
import frc.robot.subsystems.IntakeRollers.IntakeRollersIO;
import frc.robot.subsystems.IntakeRollers.IntakeRollersIOKrakenFOC;
import frc.robot.subsystems.IntakeRollers.IntakeRollersIOSim;
import frc.robot.subsystems.ShooterRollers.ShooterRollers;
import frc.robot.subsystems.ShooterRollers.ShooterRollersIO;
import frc.robot.subsystems.ShooterRollers.ShooterRollersIOKrakenFOC;
import frc.robot.subsystems.ShooterRollers.ShooterRollersIOSim;
public class RobotContainer {

	//TODO: test new shooterjoint positional pid
	//TODO: change shooter rollers to MMVelocity
	//TODO: test auto intake

	public final Drivetrain drivetrain = TunerConstants.DriveTrain;
	public final RobotState robotState = RobotState.getInstance();

	/* AdvantageKit Setup */
	public ShooterRollers shooterRollers;
	public ClimberJoint climberJoint;
	public ElevatorRollers elevatorRollers;
	public IntakeJoint intakeJoint;
	public IntakeRollers intakeRollers;
		
	private final CommandXboxController joystick = new CommandXboxController(0);
	private final GenericHID rumble = joystick.getHID();

	//Photonvision and Limelight cameras
	//PhotonVision photonVision = new PhotonVision(drivetrain,0);
	PhotonGreece photonGreece = new PhotonGreece(drivetrain);
	Limelight limelight = new Limelight();

	private SendableChooser<Command> autoChooser;

	private final Telemetry logger = new Telemetry(Constants.DriveConstants.MaxSpeed);

	public RobotContainer() {
		/* base them on Null before we beform Switch Statement check */
		shooterRollers = null;
		climberJoint = null;
		elevatorRollers = null;
		intakeJoint = null;
		intakeRollers = null;

		/* Setup according to Which Robot we are using */

		if (Constants.currentMode != Constants.Mode.REPLAY) {
			switch (Constants.currentMode) {
				case REAL:
					shooterRollers = new ShooterRollers(new ShooterRollersIOKrakenFOC());
					climberJoint = new ClimberJoint(new ClimberJointIOKrakenFOC());
					elevatorRollers = new ElevatorRollers(new ElevatorRollersIOKrakenFOC());
					intakeJoint = new IntakeJoint(new IntakeJointIOKrakenFOC());
					intakeRollers = new IntakeRollers(new IntakeRollersIOKrakenFOC());
					break;
					/* We will include the other subsystems */
				case SIM:
					shooterRollers = new ShooterRollers(new ShooterRollersIOSim());
					climberJoint = new ClimberJoint(new ClimberJointIOSim());
					elevatorRollers = new ElevatorRollers(new ElevatorRollersIOSim());
					intakeJoint = new IntakeJoint(new IntakeJointIOSim());
					intakeRollers = new IntakeRollers(new IntakeRollersIOSim());
					break;
			}
		}


		if (shooterRollers == null) {
			shooterRollers = new ShooterRollers(new ShooterRollersIO() {});
		}
		if (climberJoint == null) {
			climberJoint = new ClimberJoint(new ClimberJointIO() {});
		}
		if (elevatorRollers == null) {
			elevatorRollers = new ElevatorRollers(new ElevatorRollersIO() {});
		}
		if (intakeJoint == null) {
			intakeJoint = new IntakeJoint(new IntakeJointIO() {});
		}
		if (intakeRollers == null) {
			intakeRollers = new IntakeRollers(new IntakeRollersIO() {});
		}
		
		configureBindings();
		registerNamedCommands();
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	private void configureBindings() {
		drivetrain.setDefaultCommand(drivetrain.run(() -> drivetrain.setControllerInput(-joystick.getLeftY(),
				-joystick.getLeftX(), -joystick.getRightX())));

		drivetrain.registerTelemetry(logger::telemeterize);

	}

	private void registerNamedCommands() {

	}


    public void displaySystemInfo() {
    }

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}