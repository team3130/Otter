// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.Chassis.FlipDriveOrientation;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Intake.DumbPneumatics;
import frc.robot.commands.Intake.DumbSpintake;
import frc.robot.commands.Intake.DumbSpouttake;
import frc.robot.commands.Intake.SmartSpintake;
import frc.robot.commands.Shooter.OnlyShoot;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.VelocityShoot;
import frc.robot.commands.SpinHopper;
import frc.robot.commands.*;
import frc.robot.commands.Shooter.*;
import frc.robot.commands.Amp.AmpIntake;
import frc.robot.commands.Amp.AmpOuttake;
import frc.robot.commands.Amp.AmpPrime;
import frc.robot.commands.Amp.AmpUnPrime;
import frc.robot.commands.Chassis.FlipDriveOrientation;
import frc.robot.commands.Chassis.TeleopDrive;
import frc.robot.commands.Chassis.ZeroEverything;
import frc.robot.commands.Chassis.ZeroWheels;
import frc.robot.sensors.Camera;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Amp;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  private final Amp amp;
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  private final Shooter shooter;
  private final Indexer indexer;
  private final Intake intake;
  private final Hopper hopper;
  private final Camera camera;
  private final Chassis chassis;


  // container for the robot containing subsystems, OI devices, and commands
  public RobotContainer() {
    shooter = new Shooter();
    indexer = new Indexer();
    intake = new Intake();
    hopper = new Hopper();
    camera = new Camera();
    chassis = new Chassis(camera);
    hopper = new Hopper();
    amp = new Amp();

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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
            .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed, cancelling on release.
    // driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    new JoystickButton(driverController, Constants.Buttons.LST_BTN_X).whileTrue(new Shoot(shooter, indexer, intake));
    new POVButton(driverController, Constants.Buttons.LST_POV_N).whileTrue(new ZeroEverything(chassis));
    new POVButton(driverController, Constants.Buttons.LST_POV_W).whileTrue(new ZeroWheels(chassis));
    new JoystickButton(driverController, Constants.Buttons.LST_BTN_B).whileTrue(new FlipDriveOrientation(chassis));
    new JoystickButton(driverController, Constants.Buttons.LST_BTN_A).whileTrue(new AmpIntake(amp));
    new JoystickButton(driverController, Constants.Buttons.LST_BTN_X).whileTrue(new AmpOuttake(amp));
    new JoystickButton(driverController, Constants.Buttons.LST_BTN_X).whileTrue(new AmpPrime(amp));
    new JoystickButton(driverController, Constants.Buttons.LST_BTN_X).whileTrue(new AmpUnPrime(amp));
    new JoystickButton(driverController, Constants.Buttons.LST_BTN_Y).whileTrue(new SpinHopper(hopper));

    //new JoystickButton(driverController, Constants.Buttons.LST_BTN_B).whileTrue(new OnlyIndex(indexer));
    new JoystickButton(driverController, Constants.Buttons.LST_BTN_A).whileTrue(new OnlyShoot(shooter));
    SmartDashboard.putData(new FlipDriveOrientation(chassis));
    new JoystickButton(driverController, Constants.Buttons.LST_BTN_B).whileTrue(new FlipDriveOrientation(chassis));

    new JoystickButton(driverController, Constants.Buttons.LST_BTN_Y).whileTrue(new SpinHopper(hopper));
    new JoystickButton(driverController, Constants.Buttons.LST_BTN_B).whileTrue(new VelocityShoot(shooter));

    SmartDashboard.putData(new FlipDriveOrientation(chassis));
    new JoystickButton(operatorController, Constants.Buttons.LST_BTN_LBUMPER).whileTrue(new DumbSpouttake(new Intake()));
    new JoystickButton(operatorController, Constants.Buttons.LST_BTN_RBUMPER).whileTrue(new DumbSpintake(new Intake()));
    new JoystickButton(operatorController, Constants.Buttons.LST_BTN_A).whileTrue(new SmartSpintake(new Intake()));
    new JoystickButton(operatorController, Constants.Buttons.LST_BTN_B).whileTrue(new DumbPneumatics(new Intake()));
  }



  }