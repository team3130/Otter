// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.commands.Chassis.TeleopDrive;
import frc.robot.commands.Chassis.ZeroEverything;
import frc.robot.commands.Climber.ClimberReset;
import frc.robot.commands.Climber.PitClimberBackwards;
import frc.robot.commands.Climber.ClimberExtend;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.commands.Shooter.*;
import frc.robot.sensors.JoystickTrigger;
import frc.robot.subsystems.*;
import frc.robot.commands.Amp.*;
import frc.robot.commands.Intake.*;
import frc.robot.subsystems.Amp;
import java.util.function.BooleanSupplier;

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
  private final ShooterShifter shooterShifter;
  private final Climber leftClimber;
  private final Climber rightClimber;
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);


  // container for the robot containing subsystems, OI devices, and commands
  public RobotContainer() {


    leftClimber = new Climber(Constants.Climber.kLMotor, Constants.Climber.kLLimitSwitch, Constants.Buttons.LST_AXS_LJOYSTICKY, false);
    rightClimber = new Climber(Constants.Climber.kRMotor, Constants.Climber.kRLimitSwitch, Constants.Buttons.LST_AXS_RJOYSTICKY, false);

    shooter = new Shooter();
    shooterShifter = new ShooterShifter();
    chassis = new Chassis();
    intake = new Intake();
    amp = new Amp(operatorController);

    // Named commands must be registered before the creation of any PathPlanner Autos or Paths
    // Do this in RobotContainer, after subsystem initialization, but before the creation of any other commands.
    //NamedCommands.registerCommand("Shoot", new Shoot(shooter, intake));

    configureBindings(); // configure button bindings
    exportShuffleBoardData(); // export ShuffleBoardData

    // Default commands running in the background when other commands not scheduled
    chassis.setDefaultCommand(new TeleopDrive(chassis, driverController));
    leftClimber.setDefaultCommand(new ClimberExtend(leftClimber, operatorController));
    rightClimber.setDefaultCommand(new ClimberExtend(rightClimber, operatorController));


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
  public Command shootAuto() {
    return new Shoot(shooter, intake);
  }
  public Command setIsReset(Climber climber, boolean bool){
    return new ClimberReset(climber, bool);
  }
  public Climber getLeftClimber(){
    return leftClimber;
  }
  public Climber getRightClimber(){
    return rightClimber;
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

  public Command resetEverything() {
    return new ZeroEverything(chassis);
  }


  public void periodic() {
    operatorController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
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
    new POVButton(driverController, Constants.Buttons.LST_POV_N).whileTrue(new ZeroEverything(chassis));

    new JoystickButton(driverController, Constants.Buttons.LST_BTN_X).whileTrue(new AmpOuttake(amp));
    new JoystickButton(driverController, Constants.Buttons.LST_BTN_LBUMPER).whileTrue(new SmartSpintake(intake));
    new JoystickButton(operatorController, Constants.Buttons.LST_BTN_RBUMPER).whileTrue(new ToggleIntake(intake));
    new JoystickButton(driverController, Constants.Buttons.LST_BTN_A).whileTrue(new Outtake(intake));
    new JoystickButton(driverController, Constants.Buttons.LST_BTN_B).whileTrue(new Spintake(intake));

    new JoystickButton(operatorController, Constants.Buttons.LST_BTN_LBUMPER).whileTrue(new ToggleAmp(amp));
    new JoystickButton(operatorController, Constants.Buttons.LST_BTN_RBUMPER).whileTrue(new AmpIntake(amp));
    new JoystickButton(operatorController, Constants.Buttons.LST_BTN_A).whileTrue(new AlwaysAmpIntake(amp));

    new JoystickButton(operatorController, Constants.Buttons.LST_BTN_B).whileTrue(new OnlyIndex(shooter));
    new JoystickButton(operatorController, Constants.Buttons.LST_BTN_Y).whileTrue(new OnlyShoot(shooter));
    new JoystickButton(driverController, Constants.Buttons.LST_POV_E).whileTrue(new PitClimberBackwards(leftClimber));
    new JoystickButton(driverController, Constants.Buttons.LST_POV_S).whileTrue(new PitClimberBackwards(rightClimber));

    //new JoystickButton(operatorController, Constants.Buttons.LST_BTN_X).whileTrue(new SequentialCommandGroup(new AmpIntake(amp), new RumbleAmp(amp, operatorController)));
    // new JoystickTrigger(operatorController, Constants.Buttons.LST_BTN_B).whileTrue(new Shoot(shooter, intake));

    /*
    new POVButton(operatorController, Constants.Buttons.LST_POV_S).whileTrue(new DoubleRetract(shooterShifter));
    new POVButton(operatorController, Constants.Buttons.LST_POV_E).whileTrue(new DoubleExtend(shooterShifter));
    new POVButton(operatorController, Constants.Buttons.LST_POV_W).whileTrue(new ShifterOneExtend(shooterShifter));
    new POVButton(operatorController, Constants.Buttons.LST_POV_N).whileTrue(new ShifterTwoExtend(shooterShifter));
    */

    //new JoystickTrigger(driverController, Constants.Buttons.LST_AXS_RTRIGGER).whileTrue(new TestTrigger(intake));

    new JoystickTrigger(driverController, Constants.Buttons.LST_AXS_LTRIGGER).whileTrue(new TestTrigger(intake, driverController));
    //new JoystickButton(driverController, Constants.Buttons.LST_BTN_B).whileTrue(new VelocityShoot(shooter));
    //new JoystickButton(operatorController, Constants.Buttons.LST_BTN_Y).whileTrue(new Spintake(intake));

    //new JoystickButton(operatorController, Constants.Buttons.LST_BTN_A).whileTrue(new SmartSpintake(new Intake()));
    //new JoystickButton(operatorController, Constants.Buttons.LST_BTN_RBUMPER).whileTrue(new SequentialCommandGroup(new SmartSpintake(intake), new SmartIndex(intake)));
  }
}