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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
//import frc.robot.commands.Amp.;
//import frc.robot.commands.Amp.Software.AmpManualLift;
//import frc.robot.commands.Amp.Software.AmpManualLower;
//import frc.robot.commands.Amp.Software.AmpZero;
import frc.robot.commands.Amp.*;
import frc.robot.commands.Indexer.*;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Intake.IntakeToggle;
import frc.robot.commands.Intake.LimitedSpintake;
import frc.robot.commands.Shooter.OnlyShoot;
import frc.robot.commands.Shooter.VelocityShoot;
import frc.robot.commands.ShooterShifter.HighPosition;
import frc.robot.commands.ShooterShifter.LowestPosition;
import frc.robot.commands.ShooterShifter.ShortExtended;
import frc.robot.sensors.JoystickTrigger;
import frc.robot.subsystems.NewAmp;
import frc.robot.commands.Auto.*;
import frc.robot.commands.Chassis.TeleopDrive;
import frc.robot.commands.Climber.ClimberUnlimited;
import frc.robot.commands.Climber.PitClimberReset;
//import frc.robot.commands.Indexer.AndrewIndexToShoot;
//import frc.robot.commands.Indexer.IndexToBeam;
import frc.robot.commands.Indexer.UnlimitedOuttake;
import frc.robot.commands.Shooter.ShootMovingSetpoint;
import frc.robot.commands.Shooter.ShuttleMovingSetpoint;
//import frc.robot.commands.ShooterShifter.DoubleExtend;
//import frc.robot.commands.ShooterShifter.DoubleRetract;
//import frc.robot.commands.ShooterShifter.ShortShifterExtend;
import frc.robot.subsystems.*;
import frc.robot.commands.Chassis.ResetOdometryForward;
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
  //private final Intake intake;
  private final NewIntake intake;
  //private final Shooter shooter;
  private final NewShooter shooter;
  //private final Indexer indexer;
  private final NewIndexer indexer;
  //private final ShooterShifter shooterShifter;
  private final NewShooterShifter newShooterShifter;
  private final Climber leftClimber;
  private final Climber rightClimber;
  private final PS5Controller driverController = new PS5Controller(0);
  private final XboxController operatorController = new XboxController(1);
  private final LEDs robotLEDs;
  private final CameraSubsystem camera;
  private final SendableChooser<Command> autoChooser;
  //private final Amp amp;
  private final NewAmp newAmp;

  public RobotContainer() {
    leftClimber = new Climber(Constants.CAN.climberLeft, Constants.IDs.kLLimitSwitch, Constants.XBox.AXS_RJOYSTICK_Y, false);
    rightClimber = new Climber(Constants.CAN.climberRight, Constants.IDs.kRLimitSwitch, Constants.XBox.AXS_LJOYSTICK_Y, false);

    //shooter = new Shooter();
    shooter = new NewShooter();
    //shooterShifter = new ShooterShifter();
    newShooterShifter = new NewShooterShifter();
    chassis = new Chassis();
    //intake = new Intake();
    intake = new NewIntake();
    //indexer = new Indexer(shooter);
    indexer = new NewIndexer(shooter);
    robotLEDs = new LEDs();
    camera = new CameraSubsystem();
    //amp = new Amp();
    newAmp = new NewAmp();

    // Named commands must be registered before the creation of any PathPlanner Autos or Paths
    // Do this in RobotContainer, after subsystem initialization, but before the creation of any other commands.
    //NamedCommands.registerCommand("FlywheelsMovingSet", new AutoFlywheelMovingSetpoint(shooter));
    //NamedCommands.registerCommand("Flywheels", new AutoFlywheel(shooter));
    //NamedCommands.registerCommand("FlywheelShuttle", new AutoFlywheelShuttle(shooter));

    //NamedCommands.registerCommand("Index", new AutoIndex(indexer, shooter));
    //NamedCommands.registerCommand("IndexPreload", new AutoIndexPreload(indexer, shooter));
    //NamedCommands.registerCommand("Intake", new AutoIntake(intake, indexer));
    //NamedCommands.registerCommand("ShifterDoubleExtend", new AutoDoubleExtend(shooterShifter));
    //NamedCommands.registerCommand("ShifterShortExtend", new AutoShortExtend(shooterShifter));
    //NamedCommands.registerCommand("ShifterDoubleRetract", new AutoDoubleRetract(shooterShifter));
    //NamedCommands.registerCommand("AmpHome", new AutoAmpZero(amp));
    //NamedCommands.registerCommand("Outtake", new AutoOuttake(indexer, intake));

    configureBindings(); // configure button bindings
    exportShuffleBoardData(); // export ShuffleBoardData

    // Default commands running in the background when other commands not scheduled
    chassis.setDefaultCommand(new TeleopDrive(chassis, driverController, camera));

    leftClimber.setDefaultCommand(new ClimberUnlimited(leftClimber, operatorController));
    rightClimber.setDefaultCommand(new ClimberUnlimited(rightClimber, operatorController));

    //this.isFieldMirrored = new SendableChooser<>();
    //ally = DriverStation.getAlliance();
    //populateChooser();

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    ShuffleboardTab tab = Shuffleboard.getTab("Competition");
    tab.addBoolean("Intake Has Note", intake::getIntakeLimit).withPosition(0, 0).withSize(6, 5);
    tab.addBoolean("At Distance", camera::atShootingDistance).withPosition(6, 0).withSize(2, 2);
    tab.addBoolean("At Angle", camera::isAtYawSetpointLEDs).withPosition(8, 0).withSize(2, 2);
    tab.addBoolean("Flywheel Speed", shooter::hasShooterReachedSpeed).withPosition(6, 2).withSize(3, 2);
    tab.addBoolean("Climbers", this::climbersAreDone).withPosition(9, 2).withSize(2, 2);
    tab.addDouble("Distance #", camera::getDistanceToTarget).withPosition(6, 4).withSize(1, 1);

    tab.add("AutoChooser", autoChooser).withPosition(4, 5).withSize(4, 1);
  }

  public boolean climbersAreDone() {
    return leftClimber.brokeLimit() && rightClimber.brokeLimit();
  }

  public void exportShuffleBoardData() {
    if (Constants.debugMode) {
      ShuffleboardTab tab = Shuffleboard.getTab("Subsystem Test");
      tab.add(chassis);
      tab.add(shooter);
      tab.add(intake);
      tab.add(indexer);
      tab.add(newAmp);
      tab.add(rightClimber);
      tab.add(camera);

      chassis.exportSwerveModData(Shuffleboard.getTab("Swerve Modules"));
    }
  }

  public void LEDPeriodicRobot() {
    if (rightClimber.getClimbDone() && leftClimber.getClimbDone()) {
      robotLEDs.movingRainbow();
    } else {
    /*  LEDPeriodicBackClimbers();
      LEDPeriodicBottomFrontClimbers();
      LEDPeriodicTopFrontClimbers();
      LEDPeriodicSidebars();
      LEDPeriodicBar();
     */
    }
  }

  /*public void LEDPeriodicBackClimbers() {
      if (amp.getIsHigh()) {
        robotLEDs.setBackClimbers(Constants.LEDColors.darkGreenHSV);
      } else if (shooterShifter.getIsDoubleExtended() || (!amp.getIsHigh() && amp.ampIsAboveHeightForStage())) {
        robotLEDs.setBackClimbers(Constants.LEDColors.redHSV);
      } else if (intake.getIntakeLimitSwitch()) {
        robotLEDs.setBackClimbers(Constants.LEDColors.purpleHSV);
      } else {
        robotLEDs.setBackClimbers(Constants.LEDColors.yellowHSV);
      }
  }

  public void LEDPeriodicBottomFrontClimbers() {
    if (amp.getIsHigh()) {
      robotLEDs.setBottomFrontClimbers(Constants.LEDColors.darkGreenHSV);
    } else if (shooterShifter.getIsDoubleExtended() || (!amp.getIsHigh() && amp.ampIsAboveHeightForStage())) {
      robotLEDs.setBottomFrontClimbers(Constants.LEDColors.redHSV);
    } else if (shooter.getFlywheelAtVelocityRaw()) {
      robotLEDs.setBottomFrontClimbers(Constants.LEDColors.lightGreenHSV);
    } else if (intake.getIntakeLimitSwitch()) {
      robotLEDs.setBottomFrontClimbers(Constants.LEDColors.purpleHSV);
    } else {
      robotLEDs.setBottomFrontClimbers(Constants.LEDColors.yellowHSV);
    }
  }

  public void LEDPeriodicTopFrontClimbers() {
    if (amp.getIsHigh()) {
      robotLEDs.setTopFrontClimbers(Constants.LEDColors.darkGreenHSV);
    } else if (shooterShifter.getIsDoubleExtended() || (!amp.getIsHigh() && amp.ampIsAboveHeightForStage())) {
      robotLEDs.setTopFrontClimbers(Constants.LEDColors.redHSV);
    } else if (shooter.getBeamHasNote()) {
      robotLEDs.setTopFrontClimbers(-20);
    } else if (intake.getIntakeLimitSwitch()) {
      robotLEDs.setTopFrontClimbers(Constants.LEDColors.purpleHSV);
    } else {
      robotLEDs.setTopFrontClimbers(Constants.LEDColors.yellowHSV);
    }
  }

  public void LEDPeriodicSidebars() {
    if (amp.getIsHigh()) {
      robotLEDs.setSidebars(Constants.LEDColors.darkGreenHSV);
    } else if (shooterShifter.getIsDoubleExtended() || (!amp.getIsHigh() && amp.ampIsAboveHeightForStage())) {
      robotLEDs.setSidebars(Constants.LEDColors.redHSV);
    } else if (camera.isAtYawSetpointLEDs()) {
      robotLEDs.setSidebars(Constants.LEDColors.lightGreenHSV);
    } else if (intake.getIntakeLimitSwitch()) {
      robotLEDs.setSidebars(Constants.LEDColors.purpleHSV);
    } else {
      robotLEDs.setSidebars(Constants.LEDColors.yellowHSV);
    }
  }

  public void LEDPeriodicBar() {
      if (amp.getIsHigh()) {
        robotLEDs.setBar(Constants.LEDColors.darkGreenHSV);
      } else if (shooterShifter.getIsDoubleExtended() || (!amp.getIsHigh() && amp.ampIsAboveHeightForStage())) {
        robotLEDs.setBar(Constants.LEDColors.redHSV);
      } else if (camera.atShootingDistance()) {
        robotLEDs.setBar(Constants.LEDColors.lightGreenHSV);
      } else if (camera.attemptingToShootDistance()) {
        if (camera.getDistanceToTarget() < camera.getGoalDistanceMeters()) {
          robotLEDs.setBar(Constants.LEDColors.lightBlueHSV);
        } else {
          robotLEDs.setBar(Constants.LEDColors.orangeHSV);
        }
      } else if (intake.getIntakeLimitSwitch()) {
        robotLEDs.setBar(Constants.LEDColors.purpleHSV);
      } else {
        robotLEDs.setBar(Constants.LEDColors.yellowHSV);
      }
  }
*/
  public Command pick() {
    return autoChooser.getSelected();
  }

  public void resetClimbers() {
    leftClimber.setIsClimberReset(true);
    rightClimber.setIsClimberReset(true);
  }

  public InstantCommand resetShooterShifter() {
    return new LowestPosition(newShooterShifter);
  }

  public InstantCommand resetIntake() {
    return new IntakeIn(intake);
  }

  public Command resetAmp() { return  new AmpZeroInstant(newAmp); }

  /*public InstantCommand ampZero() {
    return new AmpZero(amp);
  }
*/
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
    new JoystickButton(driverController, Constants.PS5.BTN_CIRCLE).whileTrue(new UnlimitedSpintake(indexer));
    new JoystickButton(driverController, Constants.PS5.BTN_X).whileTrue(new UnlimitedOuttake(indexer));
    new JoystickButton(driverController, Constants.PS5.BTN_RBUMPER).whileTrue(new IntakeToggle(intake));

    //new JoystickTrigger(driverController, Constants.PS5.AXS_LTRIGGER).whileTrue(new AmpOuttake(amp));
    //new JoystickButton(driverController, Constants.PS5.BTN_SQUARE).whileTrue(new AmpIntake(amp));

    // software testing
    //new JoystickButton(driverController, Constants.PS5.BTN_TRIANGLE).whileTrue(new DriveToPID(chassis));

    /*
    ANDREW OPERATOR
    */

    // indexer
    new JoystickButton(operatorController, Constants.XBox.BTN_A).whileTrue(new IndexToBeam(indexer, newShooterShifter, shooter));
    new JoystickTrigger(operatorController, Constants.XBox.AXS_RTRIGGER).whileTrue(new AndrewIndexToShoot(indexer, newShooterShifter, shooter));

    // shooter
    //new JoystickButton(operatorController, Constants.XBox.BTN_X).whileTrue(new ShuttleMovingSetpoint(shooter));
    new JoystickButton(operatorController, Constants.XBox.BTN_X).whileTrue(new ShootMovingSetpoint(shooter));
    //new JoystickButton(operatorController, Constants.XBox.BTN_RBUMPER).whileTrue(new OnlyShoot(shooter));

    // amp
    //new POVButton(operatorController, Constants.XBox.POV_N).whileTrue(new SequentialCommandGroup(new AmpKirbyPrepMid(amp, shooterShifter), new AmpKirbyFlies(amp, shooter, shooterShifter, indexer)));
    //new POVButton(operatorController, Constants.XBox.POV_E).whileTrue(new AmpAutoHigh(amp));
    //new POVButton(operatorController, Constants.XBox.POV_S).onTrue(new AmpPIDHome(amp));
    //new POVButton(operatorController, Constants.XBox.POV_W).whileTrue(new AmpZero(amp));
    //new POVButton(operatorController, Constants.XBox.POV_N).whileTrue(new AmpGoUp(newAmp));
    //new POVButton(operatorController, Constants.XBox.POV_S).whileTrue(new AmpGoDown(newAmp));
    //new POVButton(operatorController, Constants.XBox.POV_W).whileTrue(new AmpIntake(newAmp));
    //new POVButton(operatorController,Constants.XBox.POV_W).whileTrue(new AmpOuttake(newAmp));
    //new POVButton(operatorController, Constants.XBox.POV_E).whileTrue(new AmpMidSetpoint(newAmp));
    //new POVButton(operatorController, Constants.XBox.POV_W).whileTrue(new AmpLowSetpoint(newAmp));
    //new POVButton(operatorController, Constants.XBox.BTN_B).whileTrue(new AmpHighSetpoint(newAmp));

    // shooter shifter
    new JoystickTrigger(operatorController, Constants.XBox.AXS_LTRIGGER).whileTrue(new ShortExtended(newShooterShifter));
    new JoystickButton(operatorController, Constants.XBox.BTN_LBUMPER).whileTrue(new HighPosition(newShooterShifter));
    //new JoystickButton(operatorController, Constants.XBox.BTN_B).whileTrue(new DoubleRetract(shooterShifter));


    //software debugging
    if (Constants.debugMode) {
      //new POVButton(operatorController, Constants.XBox.POV_W).whileTrue(new AmpAutoMid(amp));
      //new POVButton(operatorController, Constants.XBox.POV_S).whileTrue(new AmpAutoLow(amp));
      //new JoystickButton(operatorController, Constants.XBox.LST_BTN_RBUMPER).whileTrue(new AmpManualLift(amp));
    }

    if (Constants.pitMode) {
      new JoystickButton(operatorController, Constants.XBox.BTN_WINDOW).whileTrue(new PitClimberReset(rightClimber));//right
      new JoystickButton(operatorController, Constants.XBox.BTN_MENU).whileTrue(new PitClimberReset(leftClimber));//left
    }
  }
}


