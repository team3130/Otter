// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.commands.Chassis.TeleopDrive;
import frc.robot.commands.Chassis.ZeroEverything;
import frc.robot.commands.Chassis.ZeroWheels;
import frc.robot.commands.Climber.PitClimber;
import frc.robot.commands.Climber.ClimberExtend;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.commands.Shooter.*;
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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Camera limelight;
  private final Chassis chassis;
  private final Hopper hopper;
  private final Climber leftClimber;
  private final Climber rightClimber;
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  private final Shooter shooter;
  private final Indexer indexer;
  private final Intake intake;

  // container for the robot containing subsystems, OI devices, and commands
  public RobotContainer() {
    limelight = new Camera();
    chassis = new Chassis(limelight);
    hopper = new Hopper();

    leftClimber = new Climber(Constants.Climber.kLMotor, Constants.Climber.kLLimitSwitch, Constants.Buttons.LST_AXS_LJOYSTICKY);
    rightClimber = new Climber(Constants.Climber.kRMotor, Constants.Climber.kRLimitSwitch, Constants.Buttons.LST_AXS_RJOYSTICKY);

    shooter = new Shooter();
    indexer = new Indexer();
    intake = new Intake();

    // Named commands must be registered before the creation of any PathPlanner Autos or Paths
    // Do this in RobotContainer, after subsystem initialization, but before the creation of any other commands.

    configureBindings(); // configure button bindings
    exportShuffleBoardData(); // export ShuffleBoardData

    // Default commands running in the background when other commands not scheduled
    chassis.setDefaultCommand(new TeleopDrive(chassis, driverController));
    leftClimber.setDefaultCommand(new ClimberExtend(leftClimber, operatorController));
    rightClimber.setDefaultCommand(new ClimberExtend(rightClimber, operatorController));


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

    new JoystickButton(driverController, Constants.Buttons.LST_BTN_X).whileTrue(new Shoot(shooter, indexer, intake));
    new POVButton(driverController, Constants.Buttons.LST_POV_N).whileTrue(new ZeroEverything(chassis));
    new POVButton(driverController, Constants.Buttons.LST_POV_W).whileTrue(new ZeroWheels(chassis));
    new JoystickButton(driverController, Constants.Buttons.LST_BTN_Y).whileTrue(new SpinHopper(hopper));


    new JoystickButton(driverController, Constants.Buttons.LST_POV_E).whileTrue(new PitClimber(leftClimber));
    new JoystickButton(driverController, Constants.Buttons.LST_POV_S).whileTrue(new PitClimber(rightClimber));


    //new JoystickButton(driverController, Constants.Buttons.LST_BTN_B).whileTrue(new OnlyIndex(indexer));
    new JoystickButton(driverController, Constants.Buttons.LST_BTN_A).whileTrue(new OnlyShoot(shooter));

    new JoystickButton(driverController, Constants.Buttons.LST_BTN_B).whileTrue(new VelocityShoot(shooter));

  }
}