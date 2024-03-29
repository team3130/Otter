// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Amp.*;
import frc.robot.commands.Amp.Software.AmpZero;
import frc.robot.commands.Auto.*;
import frc.robot.commands.Chassis.TeleopDrive;
import frc.robot.commands.Climber.ClimberExtend;
import frc.robot.commands.Climber.PitClimberReset;
import frc.robot.commands.Indexer.AlwaysIndex;
import frc.robot.commands.Indexer.AndrewIndexToShoot;
import frc.robot.commands.Indexer.IndexToBeam;
import frc.robot.commands.Intake.LimitedSpintake;
import frc.robot.commands.Indexer.Outtake;
import frc.robot.commands.Shooter.ShootMovingSetpoint;
import frc.robot.commands.Shooter.ShuttleMovingSetpoint;
import frc.robot.commands.ShooterShifter.DoubleExtend;
import frc.robot.commands.ShooterShifter.ShortShifterExtend;
import frc.robot.commands.ShooterShifter.DoubleRetract;
import frc.robot.sensors.JoystickTrigger;
import frc.robot.subsystems.*;
import frc.robot.commands.Chassis.ResetOdometryForward;
import frc.robot.commands.Intake.*;
import frc.robot.subsystems.LEDs;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in robot periodic methods
 * (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

// The robot's subsystems and commands are defined here...
public class RobotContainer {
  // private final VelocityChassis velocityChassis;
  private final Chassis chassis;
  private final Intake intake;
  private final Shooter shooter;
  private final Indexer indexer;
  private final ShooterShifter shooterShifter;
  private final Climber leftClimber;
  private final Climber rightClimber;
  private final PS5Controller driverController = new PS5Controller(0);
  private final XboxController operatorController = new XboxController(1);
  private final LEDs robotLEDs;
  private final CameraSubsystem camera;
  private final SendableChooser<Command> autoChooser;
  private final Amp amp;

  public RobotContainer() {
    leftClimber = new Climber(Constants.CAN.climberLeft, Constants.IDs.kLLimitSwitch, Constants.XBox.AXS_RJOYSTICK_Y, false);
    rightClimber = new Climber(Constants.CAN.climberRight, Constants.IDs.kRLimitSwitch, Constants.XBox.AXS_LJOYSTICK_Y, true);

    shooter = new Shooter();
    shooterShifter = new ShooterShifter();
    chassis = new Chassis();
    intake = new Intake();
    indexer = new Indexer(shooter);
    robotLEDs = new LEDs();
    camera = new CameraSubsystem();
    amp = new Amp();

    // Named commands must be registered before the creation of any PathPlanner Autos or Paths
    // Do this in RobotContainer, after subsystem initialization, but before the creation of any other commands.
    NamedCommands.registerCommand("FlywheelsMovingSet", new AutoFlywheelMovingSetpoint(shooter));
    NamedCommands.registerCommand("Flywheels", new AutoFlywheel(shooter));
    NamedCommands.registerCommand("FlywheelShuttle", new AutoFlywheelShuttle(shooter));

    NamedCommands.registerCommand("Index", new AutoIndex(indexer, shooter));
    NamedCommands.registerCommand("IndexPreload", new AutoIndexPreload(indexer, shooter));
    NamedCommands.registerCommand("Intake", new AutoIntake(intake, indexer));
    NamedCommands.registerCommand("ShifterDoubleExtend", new AutoDoubleExtend(shooterShifter));
    NamedCommands.registerCommand("ShifterShortExtend", new AutoShortExtend(shooterShifter));
    NamedCommands.registerCommand("ShifterDoubleRetract", new AutoDoubleRetract(shooterShifter));
    NamedCommands.registerCommand("AmpHome", new AutoAmpHome(amp));

    configureBindings(); // configure button bindings
    exportShuffleBoardData(); // export ShuffleBoardData

    // Default commands running in the background when other commands not scheduled
    chassis.setDefaultCommand(new TeleopDrive(chassis, driverController, camera));

    leftClimber.setDefaultCommand(new ClimberExtend(leftClimber, operatorController));
    rightClimber.setDefaultCommand(new ClimberExtend(rightClimber, operatorController));

    //this.isFieldMirrored = new SendableChooser<>();
    //ally = DriverStation.getAlliance();
    //populateChooser();

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    ShuffleboardTab tab = Shuffleboard.getTab("Competition");
    tab.addBoolean("Intake Has Note", intake::getIntakeLimitSwitch).withPosition(0, 0).withSize(5, 5);
    tab.addBoolean("Shooting Distance", camera::atShootingDistance).withPosition(6, 0).withSize(5, 5);
    tab.add("AutoChooser", autoChooser).withPosition(4, 5).withSize(4, 1);

  }

  /*
  public boolean flywheelVelocitiesReady() {
    if ((shooter.getTopFlyVelocityRPS() > (shooter.getTopVelocitySetpoint() - 2)) && (shooter.getBottomFlyVelocityRPS() > (shooter.getBottomVelocitySetpoint() - 2))) {
      return true;
    } else {
      return false;
    }
  }

   */

  public void exportShuffleBoardData() {
    if (Constants.debugMode) {
      ShuffleboardTab tab = Shuffleboard.getTab("Subsystem Test");
      tab.add(chassis);
      tab.add(shooter);
      tab.add(intake);
      tab.add(indexer);
      tab.add(amp);
      tab.add(rightClimber);
      chassis.exportSwerveModData(Shuffleboard.getTab("Swerve Modules"));
    }
  }

  public void LEDPeriodic() {
    if (shooterShifter.getIsDoubleExtended()) {
      robotLEDs.redRobot();
    } else if (intake.getIntakeLimitSwitch() ) {
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
  public InstantCommand ampZero() {
    return new AmpZero(amp);
  }

  public void resetOdo() {
    chassis.resetOdometry(new Pose2d(0, 0, new Rotation2d()));
  }
  public void updateChassisPose() {
    chassis.updateOdometryFromSwerve();
  }


  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  //new Trigger(m_exampleSubsystem::exampleCondition).onTrue(new ExampleCommand(m_exampleSubsystem));

  // This method defines trigger -> command mappings
  // Triggers created via the Trigger constructor
  // CommandGenericHID subclass for CommandXboxController Xbox
  // CommandPS4Controller subclass for PS4 Controller
  // CommandJoystick for flight joysticks

  // Right joystick up == targeting amp
  // Right joystick down == targeting speaker
  // Press right joystick == targeting stage to speaker

  // face target == down on orientation stick
  private void configureBindings() {
    /*
    GAVIN DRIVER - Note: check Teleop button bindings in TeleopDrive
     */
    new POVButton(driverController, Constants.PS5.POV_N).whileTrue(new ResetOdometryForward(chassis));

    // intake / indexer
    new JoystickTrigger(driverController, Constants.PS5.AXS_RTRIGGER).whileTrue(new LimitedSpintake(intake, indexer));
    new JoystickButton(driverController, Constants.PS5.BTN_CIRCLE).whileTrue(new AlwaysIndex(indexer));
    new JoystickButton(driverController, Constants.PS5.BTN_RBUMPER).whileTrue(new ToggleIntake(intake));
    new JoystickButton(driverController, Constants.PS5.BTN_X).whileTrue(new Outtake(indexer));

    new JoystickTrigger(driverController, Constants.PS5.AXS_LTRIGGER).whileTrue(new AmpOuttake(amp));

    // software testing
    //new JoystickButton(driverController, Constants.PS5.BTN_TRIANGLE).whileTrue(new DriveToPID(chassis));

    /*
    ANDREW OPERATOR
    */

    // indexer

    new JoystickButton(operatorController, Constants.XBox.BTN_Y).whileTrue(new AlwaysIndex(indexer));
    new JoystickButton(operatorController, Constants.XBox.BTN_A).whileTrue(new IndexToBeam(indexer, shooterShifter, shooter));
    new JoystickTrigger(operatorController, Constants.XBox.AXS_RTRIGGER).whileTrue(new AndrewIndexToShoot(indexer, shooterShifter, shooter));

    // shooter
    new JoystickButton(operatorController, Constants.XBox.BTN_RBUMPER).whileTrue(new ShootMovingSetpoint(shooter));
    new JoystickButton(operatorController, Constants.XBox.BTN_X).whileTrue(new ShuttleMovingSetpoint(shooter));

    // amp
    new POVButton(operatorController, Constants.XBox.POV_S).whileTrue(new AmpHome(amp));
    new POVButton(operatorController, Constants.XBox.POV_N).whileTrue(new SequentialCommandGroup( new AmpAutoMid(amp, shooterShifter), new AmpKirbyThenFlies(amp, shooter, shooterShifter, indexer)));

    // shooter shifter
    new JoystickButton(operatorController, Constants.XBox.BTN_LBUMPER).whileTrue(new DoubleExtend(shooterShifter));
    new JoystickTrigger(operatorController, Constants.XBox.AXS_LTRIGGER).whileTrue(new ShortShifterExtend(shooterShifter));
    new POVButton(operatorController, Constants.XBox.POV_W).whileTrue(new DoubleRetract(shooterShifter));


    //software debugging
    //new POVButton(operatorController, Constants.XBox.POV_E).whileTrue(new AmpAutoHigh(amp));
    //new POVButton(operatorController, Constants.XBox.POV_W).whileTrue(new AmpAutoMid(amp));
    //new POVButton(operatorController, Constants.XBox.POV_S).whileTrue(new AmpAutoLow(amp));
    //new JoystickButton(operatorController, Constants.XBox.BTN_LBUMPER).whileTrue(new AmpHome(amp));
    //new JoystickButton(operatorController, Constants.XBox.LST_BTN_RBUMPER).whileTrue(new AmpManualLift(amp));

    new POVButton(operatorController, Constants.XBox.POV_W).whileTrue(new PitClimberReset(rightClimber));//right
    new POVButton(operatorController, Constants.XBox.POV_E).whileTrue(new PitClimberReset(leftClimber));//left
  }
}


