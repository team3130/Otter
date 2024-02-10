// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.commands.Chassis.TeleopDrive;
import frc.robot.commands.Chassis.ZeroEverything;
import frc.robot.commands.Chassis.ZeroWheels;
import frc.robot.commands.Climber.PitClimber;
import frc.robot.commands.Climber.ClimberExtend;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.commands.Shooter.*;
import frc.robot.commands.ShooterShifter.*;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Intake.*;
import frc.robot.commands.Shooter.OnlyShoot;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.subsystems.*;
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
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Chassis chassis;
  private final Climber leftClimber;
  private final Climber rightClimber;
  private final Amp amp;
  private final Shooter shooter;
  private final Intake intake;
  private final LEDSubsystem led;
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  private final SendableChooser<Command> autoChooser;
  private final ShooterShifter shooterShifter;

  // container for the robot containing subsystems, OI devices, and commands
  public RobotContainer() {

    leftClimber = new Climber(Constants.Climber.kLMotor, Constants.Climber.kLLimitSwitch, Constants.Buttons.LST_AXS_LJOYSTICKY);
    rightClimber = new Climber(Constants.Climber.kRMotor, Constants.Climber.kRLimitSwitch, Constants.Buttons.LST_AXS_RJOYSTICKY);

    led = new LEDSubsystem();
    shooter = new Shooter();
    intake = new Intake();
    chassis = new Chassis();
    amp = new Amp();
    shooterShifter = new ShooterShifter();

    // Named commands must be registered before the creation of any PathPlanner Autos or Paths
    // Do this in RobotContainer, after subsystem initialization, but before the creation of any other commands.
      // NamedCommands.registerCommand("spinHopper", hopper.spinHopperAuto());

    configureBindings(); // configure button bindings
    exportShuffleBoardData(); // export ShuffleBoardData

    // Default commands running in the background when other commands not scheduled
    chassis.setDefaultCommand(new TeleopDrive(chassis, driverController));
    led.defaultYellow();
    leftClimber.setDefaultCommand(new ClimberExtend(leftClimber, operatorController));
    rightClimber.setDefaultCommand(new ClimberExtend(rightClimber, operatorController));


    // Build an auto chooser. This will use Commands.none() as the default option.
    // autoChooser = AutoBuilder.buildAutoChooser();
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
    if (intake.getIntakeHasNote() || amp.getAmpLimitSwitch()) {
      led.greenRobot();
    } else if (intake.getNoteReadyToShoot()) {
      led.greenShooter();
    }

    if (leftClimber.getInvalidInput() || rightClimber.getInvalidInput()) {
      led.red();
    }
  }

  public void resetOdo() {
    chassis.resetOdometry(new Pose2d(0, 0, new Rotation2d()));
  }
  public Command visionShifterVelocityShoot() {
    return new SequentialCommandGroup(new VisionShift(shooterShifter), new VisionVelocityShoot(shooter));
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
    new JoystickButton(driverController, Constants.Buttons.LST_POV_E).whileTrue(new PitClimber(leftClimber));
    new JoystickButton(driverController, Constants.Buttons.LST_POV_S).whileTrue(new PitClimber(rightClimber));


    //new JoystickButton(driverController, Constants.Buttons.LST_BTN_B).whileTrue(new OnlyIndex(indexer));
    new JoystickButton(driverController, Constants.Buttons.LST_BTN_A).whileTrue(new OnlyShoot(shooter));
    new JoystickButton(operatorController, Constants.Buttons.LST_BTN_RBUMPER).whileTrue(new SequentialCommandGroup(new SmartSpintake(intake), new SmartIndex(intake)));
    new JoystickButton(driverController, Constants.Buttons.LST_BTN_X).whileTrue(new Shoot(shooter));
    //new JoystickButton(driverController, Constants.Buttons.LST_BTN_B).whileTrue(new VelocityShoot(shooter));
    new JoystickButton(driverController, Constants.Buttons.LST_BTN_X).whileTrue(visionShifterVelocityShoot());
  }



  }