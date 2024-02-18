// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import frc.robot.errlib.SlewRateLimiterSpeeds;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

/** A default command to drive in teleop based off the joysticks*/
public class TeleopDrive extends Command {
  private final boolean isFieldRelative = true;
  private final Chassis chassis;
  private final XboxController xboxController;
  private final GenericEntry n_fieldOrriented; // comp network table entry for whether field oriented drivetrain

  private final SlewRateLimiterSpeeds accelerationLimiter;
  public TeleopDrive(Chassis chassis, XboxController xboxController) {
    this.chassis = chassis;
    this.xboxController = xboxController;

    // Use addRequirements() here to declare subsystem dependencies.
    m_requirements.add(chassis);

    accelerationLimiter = new SlewRateLimiterSpeeds(
      Constants.Swerve.kMaxAccelerationDrive,
      Constants.Swerve.kMaxAccelerationAngularDrive,
      new ChassisSpeeds()
    );

    n_fieldOrriented = Shuffleboard.getTab("Chassis").add("field orriented", false).getEntry();
  }

  /**
   * Called when the scheduler first schedules the command
   */
  @Override
  public void initialize() {
    accelerationLimiter.reset(new ChassisSpeeds());
  }

  /**
   * Called periodically while the default command is being ran and is not actively interrupted.
   * Takes the Joysticks inputs, applies a slew rate limit on it in meters per second which makes the input whooshier.
   * Inverts controls if we are on the red alliance because april tags give us an absolute position of the field
   */
  @Override
  public void execute() {
    double y = -xboxController.getRawAxis(Constants.Buttons.LST_AXS_LJOYSTICKX); // left stick y-axis (y-axis is inverted)
    double x = -xboxController.getRawAxis(Constants.Buttons.LST_AXS_LJOYSTICKY); // left stick x-axis
    double theta = -xboxController.getRawAxis(Constants.Buttons.LST_AXS_RJOYSTICKX); // right stick x-axis

    // apply dead-band
    if (Math.abs(x) < Constants.Swerve.kDeadband) x = 0;
    if (Math.abs(y) < Constants.Swerve.kDeadband) y = 0;
    if (Math.abs(theta) < Constants.Swerve.kDeadband) theta = 0;
    // square the inputs
    y = y * Math.abs(y);
    x = x * Math.abs(x);
    theta = theta * Math.abs(theta);

    // convert joystick offsets to chassis speeds and apply slew rate limiter
    ChassisSpeeds newSpeeds = accelerationLimiter.calculate(new ChassisSpeeds(
      x     * Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond,
      y     * Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond,
      theta * Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond / (Constants.Swerve.trackWidth/2.0)
    ));

    if (isFieldRelative) {
      newSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(newSpeeds, chassis.getRotation2d());
    }
    chassis.drive(newSpeeds);

    n_fieldOrriented.setBoolean(isFieldRelative);
  }

  /**
   * Called when the command is over.
   * Stops the chassis modules
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    chassis.drive(new ChassisSpeeds());;
  }

  /**
   * @return false. Never is finished.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}