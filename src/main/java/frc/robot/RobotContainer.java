// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.sensors.DistanceCalcs;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.commands.Chassis.TeleopDrive;
import frc.robot.commands.Chassis.ZeroEverything;
import frc.robot.commands.Chassis.ZeroWheels;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  private final CameraSubsystem cameraSubsystem = new CameraSubsystem();
  private final Chassis chassis = new Chassis(cameraSubsystem);
  private final XboxController driverGamepad = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  protected DistanceCalcs m_distanceCalcs = new DistanceCalcs();

  public RobotContainer() {
    exportShuffleBoardData(); // export ShuffleBoardData

    chassis.setDefaultCommand(new TeleopDrive(chassis, driverGamepad, cameraSubsystem));
    configureBindings();

  }

  /** adds the subsystem {@link edu.wpi.first.util.sendable.Sendable} objects to a 'Subsystems' shuffleboard tab */
  public void exportShuffleBoardData() {
    if (Constants.debugMode) {
      ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
      tab.add(chassis);
      tab.add(cameraSubsystem);
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

    new POVButton(driverGamepad, Constants.Buttons.LST_POV_N).whileTrue(new ZeroEverything(chassis));
    new POVButton(driverGamepad, Constants.Buttons.LST_POV_W).whileTrue(new ZeroWheels(chassis));
    new JoystickButton(driverGamepad, Constants.Buttons.LST_BTN_X).onTrue(new EnableTargeting(cameraSubsystem));
    new JoystickButton(driverGamepad, Constants.Buttons.LST_BTN_A).whileTrue(new TargetingPressed(cameraSubsystem));
  }

  /*
  Sendable Commands
   */
  public Command resetEverything() {
    return new ZeroEverything(chassis);
  }
  public Command dontTarget() {
    return new DisableTargeting(cameraSubsystem);
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
