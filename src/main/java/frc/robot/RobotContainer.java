// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.commands.Chassis.TeleopDrive;
import frc.robot.commands.Chassis.ZeroEverything;
import frc.robot.commands.Amp.AlwaysAmpIntake;
import frc.robot.commands.Amp.AmpIntake;
import frc.robot.commands.Amp.AmpOuttake;
import frc.robot.commands.Amp.RumbleAmp;
import frc.robot.commands.Amp.TimedAmpIntake;
import frc.robot.commands.Amp.ToggleAmp;
import frc.robot.subsystems.Amp;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;

import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in robot periodic methods
 * (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

// The robot's subsystems and commands are defined here...
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Chassis chassis;
  private final Amp amp;
  private final Intake intake;
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);


  // container for the robot containing subsystems, OI devices, and commands
  public RobotContainer() {
    chassis = new Chassis();
    intake = new Intake();
    amp = new Amp(operatorController);

    // Named commands must be registered before the creation of any PathPlanner Autos or Paths
    // Do this in RobotContainer, after subsystem initialization, but before the creation of any other commands.
    //NamedCommands.registerCommand("Turn90Deg", new TurnToAngle(chassis, 90));

    configureBindings(); // configure button bindings
    exportShuffleBoardData(); // export ShuffleBoardData

    // Default commands running in the background when other commands not scheduled
    chassis.setDefaultCommand(new TeleopDrive(chassis, driverController));

    // Build an auto chooser. This will use Commands.none() as the default option.
    // autoChooser = AutoBuilder.buildAutoChooser();
    //autoChooser = AutoBuilder.buildAutoChooser("up");

    //SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command pick() {
    return null;//autoChooser.getSelected();
  }

  public Command getPullOut() {
    return new PathPlannerAuto("Pull out");
  }

  public Command rumbley() {
    return new RumbleAmp(amp, operatorController);
  }


  public Command resetEverything() {
    return new ZeroEverything(chassis);
  }


  public void periodic() {
      operatorController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
  }

  public void resetOdo() {
    chassis.resetOdometry(new Pose2d(0, 0, new Rotation2d()));
  }

  public void updateChassisPose() {
    chassis.updateOdometryFromSwerve();
  }

  /*
  Boolean supplier that controls when the path will be mirrored for the red alliance
  This will flip the path being followed to the red side of the field.
  THE ORIGIN WILL REMAIN ON THE BLUE SIDE
   */
  public static BooleanSupplier isFieldMirrored() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return () -> (alliance.get() == DriverStation.Alliance.Red);
    }
    return () -> false;
  }

  /**
   * adds the subsystem {@link edu.wpi.first.util.sendable.Sendable} objects to a 'Subsystems' shuffleboard tab
   */
  public void exportShuffleBoardData() {
    if (Constants.debugMode) {
      ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
      tab.add(chassis);
      tab.add(amp);
      chassis.exportSwerveModData(Shuffleboard.getTab("Swerve Modules"));
    }
  }

  // This method defines trigger -> command mappings
  // Triggers created via the Trigger constructor
  // CommandGenericHID subclass for CommandXboxController Xbox
  // CommandPS4Controller subclass for PS4 Controller
  // CommandJoystick for flight joysticks
  private void configureBindings() {
    new POVButton(driverController, Constants.Buttons.LST_POV_N).whileTrue(new ZeroEverything(chassis));
    //new JoystickButton(operatorController, Constants.Buttons.LST_BTN_X).whileTrue(new SequentialCommandGroup(new AmpIntake(amp), new RumbleAmp(amp, operatorController)));

    //new JoystickButton(operatorController, Constants.Buttons.LST_POV_S).whileTrue(new RumbleAmp(amp, operatorController));

    new JoystickButton(operatorController, Constants.Buttons.LST_BTN_LBUMPER).whileTrue(new ToggleAmp(amp));
    new JoystickButton(operatorController, Constants.Buttons.LST_BTN_RBUMPER).whileTrue(new AmpIntake(amp));
    new JoystickButton(operatorController, Constants.Buttons.LST_BTN_A).whileTrue(new AlwaysAmpIntake(amp));
    new JoystickButton(operatorController, Constants.Buttons.LST_BTN_X).whileTrue(new AmpOuttake(amp));
    new JoystickButton(operatorController, Constants.Buttons.LST_BTN_B).whileTrue(new SequentialCommandGroup(new AmpIntake(amp), new TimedAmpIntake(amp)));

    //new JoystickButton(operatorController, Constants.Buttons.LST_BTN_Y).whileTrue(new Spintake(intake));
    //new JoystickButton(operatorController, Constants.Buttons.LST_BTN_B).whileTrue(new ToggleIntake(intake));

    //new JoystickButton(operatorController, Constants.Buttons.LST_BTN_A).whileTrue(new SmartSpintake(new Intake()));
    //new JoystickButton(operatorController, Constants.Buttons.LST_BTN_RBUMPER).whileTrue(new SequentialCommandGroup(new SmartSpintake(intake), new SmartIndex(intake)));
  }
}