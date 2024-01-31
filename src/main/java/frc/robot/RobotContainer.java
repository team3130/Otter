// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.commands.Chassis.FlipDriveOrientation;
import frc.robot.commands.Chassis.TeleopDrive;
import frc.robot.commands.Chassis.ZeroEverything;
import frc.robot.commands.Chassis.ZeroWheels;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.ClimberRight;
import frc.robot.subsystems.ClimberLeft;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in robot periodic methods
 * (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

// The robot's subsystems and commands are defined here...
public class RobotContainer {
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Camera limelight;
  private final Chassis chassis;
  private final Hopper hopper;
  private final ClimberRight climberRight;
  private final ClimberLeft climberLeft;
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  private final SendableChooser<Command> autoChooser;

  // container for the robot containing subsystems, OI devices, and commands
  public RobotContainer() {
    limelight = new Camera();
    chassis = new Chassis(limelight);
    hopper = new Hopper();
    climberRight = new ClimberRight();
    climberLeft = new ClimberLeft();

    // Named commands must be registered before the creation of any PathPlanner Autos or Paths
    // Do this in RobotContainer, after subsystem initialization, but before the creation of any other commands.
    NamedCommands.registerCommand("spinHopper", hopper.spinHopperAuto());

    configureBindings(); // configure button bindings
    exportShuffleBoardData(); // export ShuffleBoardData

    // Default commands running in the background when other commands not scheduled
    chassis.setDefaultCommand(new TeleopDrive(chassis, driverController));

    // Build an auto chooser. This will use Commands.none() as the default option.
    // autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    autoChooser = AutoBuilder.buildAutoChooser("up");

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command pick() {
    return autoChooser.getSelected();
  }

  public Command getPullOut() {
    return new PathPlannerAuto("Pull out");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  public void periodic() {

  }

  /** adds the subsystem {@link edu.wpi.first.util.sendable.Sendable} objects to a 'Subsystems' shuffleboard tab */
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

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed, cancelling on release.
    // driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    new POVButton(driverController, Constants.Buttons.LST_POV_N).whileTrue(new ZeroEverything(chassis));
    new POVButton(driverController, Constants.Buttons.LST_POV_W).whileTrue(new ZeroWheels(chassis));
    new JoystickButton(driverController, Constants.Buttons.LST_BTN_B).whileTrue(new FlipDriveOrientation(chassis));
    new JoystickButton(driverController, Constants.Buttons.LST_BTN_Y).whileTrue(new SpinHopper(hopper));

    SmartDashboard.putData(new FlipDriveOrientation(chassis));
  }

  /*
  Sendable Commands
   */
  public Command resetEverything() {
    return new ZeroEverything(chassis);
  }

  /*
  Odometry and Chassis methods
   */
  public void resetOdometryWithoutApril() {
    chassis.resetOdometry(new Pose2d(0, 0, new Rotation2d()));
  }

  public void updateChassisPose() {
    chassis.updateOdometery();
  }
}
