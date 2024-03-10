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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Chassis.TeleopDrive;
import frc.robot.commands.Chassis.TuneTeleopDrive;
import frc.robot.commands.Chassis.ZeroEverything;
import frc.robot.commands.Chassis.ZeroWheels;
import frc.robot.commands.SpinHopper;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in robot periodic methods
 * (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

// The robot's subsystems and commands are defined here...
public class RobotContainer {
 private final Chassis chassis;
//  private final Amp amp;
//  private final Intake intake;
//  private final Shooter shooter;
//  private final Indexer indexer;
//  private final ShooterShifter shooterShifter;
 // private final Climber leftClimber;
  //private final Climber rightClimber;
  private final Hopper hopper;
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
  //private final LEDs robotLEDs;
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    //leftClimber = new Climber(Constants.CAN.climberLeft, Constants.IDs.kLLimitSwitch, Constants.XBox.LST_AXS_RJOYSTICKY, false);
    //rightClimber = new Climber(Constants.CAN.climberRight, Constants.IDs.kRLimitSwitch, Constants.XBox.LST_AXS_LJOYSTICKY, false);

//    shooter = new Shooter();
//    shooterShifter = new ShooterShifter();
      chassis = new Chassis();
      hopper = new Hopper();
//    amp = new Amp();
//    intake = new Intake();
//    indexer = new Indexer();
//    robotLEDs = new LEDs();

    // Named commands must be registered before the creation of any PathPlanner Autos or Paths
    // Do this in RobotContainer, after subsystem initialization, but before the creation of any other commands.
//    NamedCommands.registerCommand("Shoot", new AutoShoot(shooter, indexer));
//    NamedCommands.registerCommand("PreloadShoot", new AutoPreloadShoot(shooter, indexer));
//    NamedCommands.registerCommand("Intake", new AutoIntake(intake, indexer));
//    NamedCommands.registerCommand("ShiftDoubleExtend", new AutoDoubleExtend(shooterShifter));
//    NamedCommands.registerCommand("ShiftDoubleRetract", new AutoDoubleRetract(shooterShifter));
//    NamedCommands.registerCommand("ShiftShortExtend", new AutoMidShifter(shooterShifter));
//    NamedCommands.registerCommand("Flywheel", new AutoFlywheel(shooter));
//    NamedCommands.registerCommand("Index", new AutoIndexer(indexer));

    configureBindings(); // configure button bindings
    exportShuffleBoardData(); // export ShuffleBoardData

    // Default commands running in the background when other commands not scheduled
    chassis.setDefaultCommand(new TeleopDrive(chassis, driverController));

    //leftClimber.setDefaultCommand(new ClimberExtend(leftClimber, operatorController));
    //rightClimber.setDefaultCommand(new ClimberExtend(rightClimber, operatorController));

    //this.isFieldMirrored = new SendableChooser<>();
    //ally = DriverStation.getAlliance();
    //populateChooser();

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    ShuffleboardTab tab = Shuffleboard.getTab("Competition");
    //tab.addBoolean("Intake Has Note", intake::getIntakeLimitSwitch).withPosition(0, 0).withSize(13, 5);
    tab.add("AutoChooser", autoChooser).withPosition(4, 5).withSize(4, 1);


  }

  public void exportShuffleBoardData() {
    if (Constants.debugMode) {
      ShuffleboardTab tab = Shuffleboard.getTab("Subsystem Test");
      tab.add(chassis);
//      tab.add(shooter);
//      tab.add(intake);
//      tab.add(leftClimber);
//      tab.add(robotLEDs);
//      tab.add(indexer);
      chassis.exportSwerveModData(Shuffleboard.getTab("Swerve Modules"));
    }
  }
 /*
  public void LEDPeriodic() {
    if (shooterShifter.getIsDoubleExtended()) {
      robotLEDs.redRobot();
    } else if (intake.getIntakeLimitSwitch() || amp.getLimitSwitch()) {
      robotLEDs.purpleRobot();
    } else if (leftClimber.getClimbDone() && rightClimber.getClimbDone()) {
      robotLEDs.movingRainbow();
    } else {
      robotLEDs.yellowRobot();
    }
  }

  public Command pick() {
    return autoChooser.getSelected();
  }

  public void resetClimbers() {
    leftClimber.setIsClimberReset(true);
    rightClimber.setIsClimberReset(true);
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

  */

  public void resetOdo() {
    chassis.resetOdometry(new Pose2d(0, 0, new Rotation2d()));
  }
  public void updateChassisPose() {
    chassis.updateOdometryFromSwerve();
  }
  
  // This method defines trigger -> command mappings
  // Triggers created via the Trigger constructor
  // CommandGenericHID subclass for CommandXboxController Xbox
  // CommandPS4Controller subclass for PS4 Controller
  // CommandJoystick for flight joysticks
  private void configureBindings() {

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed, cancelling on release.
    // driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    new POVButton(driverController, Constants.XBox.LST_POV_N).whileTrue(new ZeroEverything(chassis));
    new POVButton(driverController, Constants.XBox.LST_POV_W).whileTrue(new ZeroWheels(chassis));
    new JoystickButton(driverController, Constants.XBox.LST_BTN_B).whileTrue(new SpinHopper(hopper));

    new JoystickButton(driverController, Constants.XBox.LST_BTN_Y).whileTrue(new TuneTeleopDrive(chassis));

  }
}