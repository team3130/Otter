// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Chassis.TeleopDrive;
import frc.robot.commands.Climber.ClimberExtend;
import frc.robot.commands.Indexer.AlwaysIndex;
import frc.robot.commands.Intake.LimitSpintake;
import frc.robot.commands.Indexer.Outtake;
import frc.robot.commands.LEDs.WhiteLEDs;
import frc.robot.commands.ShooterShifter.ShortShifterExtend;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.commands.Auton.*;
import frc.robot.commands.Shooter.*;
import frc.robot.commands.ShooterShifter.DoubleExtend;
import frc.robot.commands.ShooterShifter.DoubleRetract;
import frc.robot.sensors.JoystickTrigger;
import frc.robot.subsystems.*;
import frc.robot.commands.Amp.*;
import frc.robot.commands.Chassis.ResetOdometry;
import frc.robot.commands.Intake.*;
import frc.robot.subsystems.Amp;
import frc.robot.subsystems.LEDs.LEDs;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in robot periodic methods
 * (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

// The robot's subsystems and commands are defined here...
public class RobotContainer {
  private final Chassis chassis;
  private final Amp amp;
  private final Intake intake;
  private final Shooter shooter;
  private final Indexer indexer;
  private final ShooterShifter shooterShifter;
  private final Climber leftClimber;
  private final Climber rightClimber;
  private final PS5Controller driverController = new PS5Controller(0);
  private final XboxController operatorController = new XboxController(1);
  //private final AmpLEDs ampLEDs;
  private final LEDs robotLEDs;
  private final SendableChooser<Command> autoChooser;
  //private static SendableChooser<BooleanSupplier> isFieldMirrored = new SendableChooser<>();
  //private static Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();



  // container for the robot containing subsystems, OI devices, and commands
  public RobotContainer() {
    leftClimber = new Climber(Constants.CAN.climberLeft, Constants.IDs.kLLimitSwitch, Constants.XBox.LST_AXS_RJOYSTICKY, false);
    rightClimber = new Climber(Constants.CAN.climberRight, Constants.IDs.kRLimitSwitch, Constants.XBox.LST_AXS_LJOYSTICKY, false);

    shooter = new Shooter();
    shooterShifter = new ShooterShifter();
    chassis = new Chassis();
    amp = new Amp();
    intake = new Intake();
    indexer = new Indexer();

    //ampLEDs = new AmpLEDs();
    robotLEDs = new LEDs(0, 23);

    // Named commands must be registered before the creation of any PathPlanner Autos or Paths
    // Do this in RobotContainer, after subsystem initialization, but before the creation of any other commands.
    NamedCommands.registerCommand("Shoot", new AutoShoot(shooter, indexer));
    NamedCommands.registerCommand("PreloadShoot", new AutoPreloadShoot(shooter, indexer));
    NamedCommands.registerCommand("Intake", new AutoIntake(intake, indexer));
    NamedCommands.registerCommand("ShiftDoubleExtend", new AutonDoubleExtend(shooterShifter));
    NamedCommands.registerCommand("ShiftDoubleRetract", new AutonDoubleRetract(shooterShifter));
    NamedCommands.registerCommand("ShiftShortExtend", new AutonMidShifter(shooterShifter));
    NamedCommands.registerCommand("Flywheel", new AutoFlywheel(shooter));
    NamedCommands.registerCommand("Index", new AutoIndexer(indexer));

    configureBindings(); // configure button bindings
    exportShuffleBoardData(); // export ShuffleBoardData

    // Default commands running in the background when other commands not scheduled
    chassis.setDefaultCommand(new TeleopDrive(chassis, driverController));
    leftClimber.setDefaultCommand(new ClimberExtend(leftClimber, operatorController));
    rightClimber.setDefaultCommand(new ClimberExtend(rightClimber, operatorController));

    //this.isFieldMirrored = new SendableChooser<>();
    //ally = DriverStation.getAlliance();
    //populateChooser();

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser("ShootLoadedIntakeTop");
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void resetClimbers(){
    leftClimber.setIsClimberReset(true);
    rightClimber.setIsClimberReset(true);
  }

  public Command pick() {
    return autoChooser.getSelected();
  }

  public Command resetOdometry() {
    return new ResetOdometry(chassis);
  }

  public InstantCommand resetShooterShifter() {
    return new DoubleRetract(shooterShifter);
  }

  public InstantCommand resetIntake() {
    return new IntakeIn(intake);
  }

  public InstantCommand resetAmp() {
    return new AmpDown(amp);
  }

  public void LEDPeriodic() {
    if (intake.getIntakeLimitSwitch() || amp.getLimitSwitch()) {
      robotLEDs.purpleRobot();
    } else if (leftClimber.getClimbDone() && rightClimber.getClimbDone()) {
      robotLEDs.movingRainbow();
    } else {
      robotLEDs.shooterYellow();
    }

    if (robotLEDs.getWhiteLED()) {
      robotLEDs.greenRobot();
    }
  }


  public void resetOdo() {
    chassis.resetOdometry(new Pose2d(0, 0, new Rotation2d()));
  }

  /*
  public Command visionShifterVelocityShoot() {
    return new SequentialCommandGroup(new VisionShift(shooterShifter), new VisionVelocityShoot(shooter));
  }
  */

  public void updateChassisPose() {
    chassis.updateOdometryFromSwerve();
  }

  /**
   * adds the subsystem {@link edu.wpi.first.util.sendable.Sendable} objects to a 'Subsystems' shuffleboard tab
   */
  public void exportShuffleBoardData() {
    if (Constants.debugMode) {
      ShuffleboardTab tab = Shuffleboard.getTab("Subsystem Test");
      tab.add(chassis);
      tab.add(shooter);
      tab.add(intake);
      tab.add(leftClimber);
      tab.add(robotLEDs);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous

  public Command getAutonomousCommand() {
  // An example command will be run in autonomous
  return Autos.exampleAuto(m_exampleSubsystem);
  }
   */

  // This method defines trigger -> command mappings
  // Triggers created via the Trigger constructor
  // CommandGenericHID subclass for CommandXboxController Xbox
  // CommandPS4Controller subclass for PS4 Controller
  // CommandJoystick for flight joysticks
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    /*
    new Trigger(m_exampleSubsystem::exampleCondition)
            .onTrue(new ExampleCommand(m_exampleSubsystem));
    */

    // GAVIN DRIVER
    new POVButton(driverController, Constants.PS5.LST_POV_N).whileTrue(new ResetOdometry(chassis));
    new JoystickTrigger(driverController, Constants.PS5.LST_AXS_LTRIGGER).whileTrue(new AmpOuttake(amp));
    new JoystickTrigger(driverController, Constants.PS5.LST_AXS_RTRIGGER).whileTrue(new LimitSpintake(intake, indexer));
    new JoystickButton(driverController, Constants.PS5.LST_BTN_RBUMPER).whileTrue(new ToggleIntakeIn(intake));
    new JoystickButton(driverController, Constants.PS5.x).whileTrue(new Outtake(indexer));
    new JoystickButton(driverController, Constants.PS5.circle).whileTrue(new AlwaysIndex(indexer));
    //new POVButton(driverController, Constants.PS5.LST_POV_S).whileTrue(new WhiteLEDs(robotLEDs));

    // ANDREW OPERATOR
    new JoystickButton(operatorController, Constants.XBox.LST_BTN_Y).whileTrue(new ToggleAmp(amp));
    new JoystickButton(operatorController, Constants.XBox.LST_BTN_B).whileTrue(new AmpIntake(amp));
    new JoystickButton(operatorController, Constants.XBox.LST_BTN_A).whileTrue(new AlwaysAmpIntake(amp));

    new JoystickButton(operatorController, Constants.XBox.LST_BTN_RBUMPER).whileTrue(new OnlyShoot(shooter));
    new JoystickTrigger(operatorController, Constants.XBox.LST_AXS_RTRIGGER).whileTrue(new AlwaysIndex(indexer));

    new JoystickButton(operatorController, Constants.XBox.LST_BTN_LBUMPER).whileTrue(new DoubleExtend(shooterShifter));
    new POVButton(operatorController, Constants.XBox.LST_POV_N).whileTrue(new DoubleRetract(shooterShifter));
    new JoystickTrigger(operatorController, Constants.XBox.LST_AXS_LTRIGGER).whileTrue(new ShortShifterExtend(shooterShifter)); // correct

    //new JoystickButton(driverController, Constants.Buttons.LST_BTN_B).whileTrue(new VelocityShoot(shooter));
    //new JoystickButton(operatorController, Constants.Buttons.LST_BTN_RBUMPER).whileTrue(new SequentialCommandGroup(new SmartSpintake(intake), new SmartIndex(intake)));
  }
}