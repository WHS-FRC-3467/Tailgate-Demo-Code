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
import frc.robot.RobotState.TARGET;
import frc.robot.Util.LaserCanSensor;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.IntakeJoint.IntakeJoint;
import frc.robot.subsystems.IntakeJoint.IntakeJointIO;
import frc.robot.subsystems.IntakeJoint.IntakeJointIOPneumaticFOC;
import frc.robot.subsystems.IntakeJoint.IntakeJointIOSim;
import frc.robot.subsystems.IntakeRollers.IntakeRollers;
import frc.robot.subsystems.IntakeRollers.IntakeRollersIO;
import frc.robot.subsystems.IntakeRollers.IntakeRollersIOKrakenFOC;
import frc.robot.subsystems.ShooterHood.ShooterHood;
import frc.robot.subsystems.ShooterHood.ShooterHoodIO;
import frc.robot.subsystems.ShooterHood.ShooterHoodIOPneumaticFOC;
import frc.robot.subsystems.ShooterHood.ShooterHoodIOSim;
import frc.robot.subsystems.IntakeRollers.IntakeRollersIOSim;
import frc.robot.subsystems.ShooterRollers.ShooterRollers;
import frc.robot.subsystems.ShooterRollers.ShooterRollersIO;
import frc.robot.subsystems.ShooterRollers.ShooterRollersIOKrakenFOC;
import frc.robot.subsystems.ShooterRollers.ShooterRollersIOSim;
import frc.robot.subsystems.Tower.Tower;
import frc.robot.subsystems.Tower.TowerIO;
import frc.robot.subsystems.Tower.TowerIOKrakenFOC;
import frc.robot.subsystems.Tower.TowerIOSim;
public class RobotContainer {

	// TODO: Tune Drivebase tuner constants on phoenix tuner

	public final Drivetrain drivetrain = TunerConstants.DriveTrain;
	public final RobotState robotState = RobotState.getInstance();

	/* AdvantageKit Setup */
	public ShooterRollers shooterRollers;
	public Tower tower;
	public IntakeJoint intakeJoint;
	public IntakeRollers intakeRollers;
	public ShooterHood shooterHood;
		
	private final CommandXboxController joystick = new CommandXboxController(0);
	private final GenericHID rumble = joystick.getHID();

	//Photonvision and Limelight cameras
	//PhotonVision photonVision = new PhotonVision(drivetrain,0);
	PhotonGreece photonGreece = new PhotonGreece(drivetrain);
	Limelight limelight = new Limelight();

	// Triggers stuff
	private Trigger towerFullTrigger = new Trigger(() -> tower.getStatus() == Tower.TowerStatus.MIDDLEANDUPPER);
	private Trigger towerEmptyTrigger = new Trigger(() -> tower.getStatus() == Tower.TowerStatus.NOBALLS);

	private SendableChooser<Command> autoChooser;

	private final Telemetry logger = new Telemetry(Constants.DriveConstants.MaxSpeed);

	public RobotContainer() {
		/* base them on Null before we beform Switch Statement check */
		shooterRollers = null;
		tower = null;
		intakeJoint = null;
		intakeRollers = null;

		/* Setup according to Which Robot we are using */

		if (Constants.currentMode != Constants.Mode.REPLAY) {
			switch (Constants.currentMode) {
				case REAL:
					shooterRollers = new ShooterRollers(new ShooterRollersIOKrakenFOC());
					tower = new Tower(new TowerIOKrakenFOC());
					intakeJoint = new IntakeJoint(new IntakeJointIOPneumaticFOC());
					intakeRollers = new IntakeRollers(new IntakeRollersIOKrakenFOC());
					break;
					/* We will include the other subsystems */
				case SIM:
					shooterRollers = new ShooterRollers(new ShooterRollersIOSim());
					tower = new Tower(new TowerIOSim());
					intakeJoint = new IntakeJoint(new IntakeJointIOSim());
					intakeRollers = new IntakeRollers(new IntakeRollersIOSim());
					break;
			}
		}


		if (shooterRollers == null) {
			shooterRollers = new ShooterRollers(new ShooterRollersIO() {});
		}
		if (tower == null) {
			tower = new Tower(new TowerIO() {});
		}
		if (intakeJoint == null) {
			intakeJoint = new IntakeJoint(new IntakeJointIO() {});
		}
		if (intakeRollers == null) {
			intakeRollers = new IntakeRollers(new IntakeRollersIO() {});
		}
		if (shooterHood == null) {
			shooterHood = new ShooterHood(new ShooterHoodIO() {});
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

		// Intake - Left Trigger
		joystick.leftTrigger().whileTrue(Commands.deadline(
			intakeJoint.setStateCommand(IntakeJoint.State.EXTENDED), 
			intakeRollers.setStateCommand(IntakeRollers.State.INTAKE),
			tower.setStateCommand(Tower.State.INTAKE),
			Commands.waitUntil(towerFullTrigger))); // Trigger that gets this from elevator status
		// If tower is full, let driver know
		joystick.leftTrigger().and(towerFullTrigger).whileTrue(Commands.startEnd(() -> rumble.setRumble(GenericHID.RumbleType.kBothRumble, 1), () -> rumble.setRumble(GenericHID.RumbleType.kBothRumble, 0)));
		// Shuffle balls up tower (no intake)
		joystick.leftBumper().whileTrue(Commands.deadline(
			tower.setStateCommand(Tower.State.INTAKE),
			Commands.waitUntil(towerFullTrigger))); // Trigger that gets this from elevator status
		// If tower is full, let driver know
		joystick.leftBumper().and(towerFullTrigger).whileTrue(Commands.startEnd(() -> rumble.setRumble(GenericHID.RumbleType.kBothRumble, 1), () -> rumble.setRumble(GenericHID.RumbleType.kBothRumble, 0)));

		// Shoot Lower hub
		joystick.a().whileTrue(Commands.deadline(
			shooterRollers.setStateCommand(ShooterRollers.State.LOWERHUB),
			shooterHood.setStateCommand(ShooterHood.State.FORWARD),
			tower.setStateCommand(Tower.State.SHOOT),
			Commands.waitUntil(towerEmptyTrigger)
		));
		// Shoot Upper hub
		joystick.b().whileTrue(Commands.deadline(
			shooterRollers.setStateCommand(ShooterRollers.State.UPPERHUB),
			shooterHood.setStateCommand(ShooterHood.State.REVERSE),	
			tower.setStateCommand(Tower.State.SHOOT),
			Commands.waitUntil(towerEmptyTrigger)
		));
		// Shoot Tarmac
		joystick.y().whileTrue(Commands.deadline(
			shooterRollers.setStateCommand(ShooterRollers.State.TARMAC),
			shooterHood.setStateCommand(ShooterHood.State.FORWARD),	
			tower.setStateCommand(Tower.State.SHOOT),
			Commands.waitUntil(towerEmptyTrigger)
		));
		// Shoot Launchpad
		joystick.x().whileTrue(Commands.deadline(
			shooterRollers.setStateCommand(ShooterRollers.State.LAUNCHPAD),
			shooterHood.setStateCommand(ShooterHood.State.FORWARD),	
			tower.setStateCommand(Tower.State.SHOOT),
			Commands.waitUntil(towerEmptyTrigger)
		));
	}

	private void registerNamedCommands() {

	}


    public void displaySystemInfo() {
    }

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}