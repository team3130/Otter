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
import frc.robot.commands.Chassis.DriveVelocityTuning;
import frc.robot.commands.Chassis.TeleopDrive;
import frc.robot.commands.Climber.ClimberExtend;
import frc.robot.commands.Climber.PitClimberReset;
import frc.robot.subsystems.Chassis;
import frc.robot.sensors.JoystickTrigger;
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
  private final Chassis chassis;
  private final Shooter shooter;
  private final Indexer indexer;
  private final Climber leftClimber;
  private final Climber rightClimber;
  private final PS5Controller driverController = new PS5Controller(0);
  private final XboxController operatorController = new XboxController(1);
  private final LEDs robotLEDs;
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    leftClimber = new Climber(Constants.CAN.climberLeft, Constants.IDs.kLLimitSwitch, Constants.XBox.LST_AXS_RJOYSTICKY, false);
    rightClimber = new Climber(Constants.CAN.climberRight, Constants.IDs.kRLimitSwitch, Constants.XBox.LST_AXS_LJOYSTICKY, false);

    shooter = new Shooter();
    chassis = new Chassis();
    indexer = new Indexer(shooter);
    robotLEDs = new LEDs();

    // Named commands must be registered before the creation of any PathPlanner Autos or Paths
    // Do this in RobotContainer, after subsystem initialization, but before the creation of any other commands.

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
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    ShuffleboardTab tab = Shuffleboard.getTab("Competition");
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
      tab.add(leftClimber);
      tab.add(robotLEDs);
      tab.add(indexer);
      chassis.exportSwerveModData(Shuffleboard.getTab("Swerve Modules"));
    }
  }

  public void LEDPeriodic() {
  }

  public Command pick() {
    return autoChooser.getSelected();
  }

  public void resetClimbers() {
    leftClimber.setIsClimberReset(true);
    rightClimber.setIsClimberReset(true);
  }

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
    /*
    GAVIN DRIVER - Note: check Teleop button bindings in TeleopDrive
     */
    new POVButton(driverController, Constants.PS5.LST_POV_N).whileTrue(new ResetOdometryForward(chassis));
    // Right joystick up == targeting amp
    // Right joystick down == targeting speaker
    // Press right joystick == targeting stage to speaker
    new JoystickButton(driverController, Constants.PS5.triangle).whileTrue(new DriveVelocityTuning(chassis));

    //new JoystickTrigger(driverController, Constants.PS5.LST_AXS_LTRIGGER).whileTrue(new AmpOuttake(amp));

    /*
    ANDREW OPERATOR
     */

    // new JoystickButton(operatorController, Constants.XBox.LST_BTN_Y).whileTrue(new ToggleAmp(amp));
    //new JoystickButton(operatorController, Constants.XBox.LST_BTN_B).whileTrue(new AmpIntake(amp));
    //new JoystickButton(operatorController, Constants.XBox.LST_BTN_A).whileTrue(new AlwaysAmpIntake(amp));


    new POVButton(operatorController, Constants.XBox.LST_POV_W).whileTrue(new PitClimberReset(rightClimber));//right
    new POVButton(operatorController, Constants.XBox.LST_POV_E).whileTrue(new PitClimberReset(leftClimber));//left
  }
  /*
    //new JoystickButton(driverController, Constants.Buttons.LST_BTN_B).whileTrue(new VelocityShoot(shooter));
    //new JoystickButton(operatorController, Constants.Buttons.LST_BTN_RBUMPER).whileTrue(new SequentialCommandGroup(new SmartSpintake(intake), new SmartIndex(intake)));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    /*
    new Trigger(m_exampleSubsystem::exampleCondition)
            .onTrue(new ExampleCommand(m_exampleSubsystem));
    */
}