// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.Chassis.FlipDriveOrientation;
import frc.robot.commands.Chassis.TeleopDrive;
import frc.robot.commands.Chassis.ZeroEverything;
import frc.robot.commands.Chassis.ZeroWheels;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Intake.*;
import frc.robot.commands.Shooter.Handoff;
import frc.robot.subsystems.*;

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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  private final Intake intake;
  private final Indexer indexer;
  private final Chassis chassis;
  //private final SendableChooser<Command> autoChooser;

  private final CameraSubsystem camera;


  // container for the robot containing subsystems, OI devices, and commands
  public RobotContainer() {
    indexer = new Indexer();
    intake = new Intake();
    camera = new CameraSubsystem();
    chassis = new Chassis();

    // Named commands must be registered before the creation of any PathPlanner Autos or Paths
    // Do this in RobotContainer, after subsystem initialization, but before the creation of any other commands.
    NamedCommands.registerCommand("ZeroEverything", new ZeroEverything(chassis));

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


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  public Command resetEverything() {
    return new ZeroEverything(chassis);
  }


  public void periodic() {

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
      chassis.exportSwerveModData(Shuffleboard.getTab("Swerve Modules"));
    }
  }

  // This method defines trigger -> command mappings
  // Triggers created via the Trigger constructor
  // CommandGenericHID subclass for CommandXboxController Xbox
  // CommandPS4Controller subclass for PS4 Controller
  // CommandJoystick for flight joysticks
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
            .onTrue(new ExampleCommand(m_exampleSubsystem));

    new JoystickButton(driverController, Constants.Buttons.LST_BTN_A).whileTrue(new ZeroWheels(chassis));
    new JoystickButton(driverController, Constants.Buttons.LST_BTN_Y).whileTrue(new FlipDriveOrientation(chassis));
    new POVButton(driverController, Constants.Buttons.LST_POV_N).whileTrue(new ZeroEverything(chassis));


    //new JoystickButton(driverController, Constants.Buttons.LST_BTN_B).whileTrue(new OnlyIndex(indexer));

    SmartDashboard.putData(new FlipDriveOrientation(chassis));
    new JoystickButton(driverController, Constants.Buttons.LST_BTN_B).whileTrue(new FlipDriveOrientation(chassis));
    new POVButton(driverController, Constants.Buttons.LST_POV_N).whileTrue(new ZeroEverything(chassis));
    //new JoystickButton(driverController, Constants.Buttons.LST_BTN_X).whileTrue(new TurnToAngle(chassis, 90));
    SmartDashboard.putData(new FlipDriveOrientation(chassis));
    new JoystickButton(operatorController, Constants.Buttons.LST_BTN_LBUMPER).whileTrue(new DumbSpouttake(new Intake()));
    new JoystickButton(operatorController, Constants.Buttons.LST_BTN_RBUMPER).whileTrue(new DumbSpintake(new Intake()));
    new JoystickButton(operatorController, Constants.Buttons.LST_BTN_A).whileTrue(new SmartSpintake(new Intake()));
    new JoystickButton(operatorController, Constants.Buttons.LST_BTN_B).whileTrue(new DumbPneumatics(new Intake()));
    new JoystickButton(operatorController, Constants.Buttons.LST_AXS_LTRIGGER).whileTrue(new Handoff(new Indexer(), new Intake()));
    new JoystickButton(operatorController, Constants.Buttons.LST_AXS_RTRIGGER).whileTrue(new IntakeThroughIndexer(new Indexer(), new Intake()));
  }

}