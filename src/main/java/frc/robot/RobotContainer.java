// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.Shooter.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in robot periodic methods
 * (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

// The robot's subsystems and commands are defined here...
public class RobotContainer {
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  private final Shooter shooter;
  private final Indexer indexer;
  private final Intake intake;

  // container for the robot containing subsystems, OI devices, and commands
  public RobotContainer() {
    shooter = new Shooter();
    indexer = new Indexer();
    intake = new Intake();

    // Named commands must be registered before the creation of any PathPlanner Autos or Paths
    // Do this in RobotContainer, after subsystem initialization, but before the creation of any other commands.

    configureBindings(); // configure button bindings
    exportShuffleBoardData(); // export ShuffleBoardData

    // Default commands running in the background when other commands not scheduled

    // Build an auto chooser. This will use Commands.none() as the default option.
    // autoChooser = AutoBuilder.buildAutoChooser();

    // An
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

  public void periodic() {

  }

  /**
   * adds the subsystem {@link edu.wpi.first.util.sendable.Sendable} objects to a 'Subsystems' shuffleboard tab
   */
  public void exportShuffleBoardData() {
    if (Constants.debugMode) {
      ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
      tab.add(shooter);
      tab.add(intake);
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

    new JoystickButton(driverController, Constants.Buttons.LST_BTN_X).whileTrue(new Shoot(shooter, indexer, intake));

    //new JoystickButton(driverController, Constants.Buttons.LST_BTN_B).whileTrue(new OnlyIndex(indexer));
    new JoystickButton(driverController, Constants.Buttons.LST_BTN_A).whileTrue(new OnlyShoot(shooter));

    new JoystickButton(driverController, Constants.Buttons.LST_BTN_B).whileTrue(new VelocityShoot(shooter));

  }
}