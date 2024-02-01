// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

/** A default command to drive in teleop based off the joysticks*/
public class TeleopDrive extends Command {
  private final Chassis chassis;
  private final XboxController xboxController;

  // acceleration limiters for the x dimension, y dimension, and the holonomic rotation.
  // These values are in m/s and rad/s respectively
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  public TeleopDrive(Chassis chassis, XboxController xboxController) {
    this.chassis = chassis;
    this.xboxController = xboxController;
    // Use addRequirements() here to declare subsystem dependencies.
    m_requirements.add(chassis);

    // Initializes slew rate limiters to limit acceleration
    xLimiter = new SlewRateLimiter(Constants.Swerve.kMaxAccelerationDrive);
    yLimiter = new SlewRateLimiter(Constants.Swerve.kMaxAccelerationDrive);
    turningLimiter = new SlewRateLimiter(Constants.Swerve.kMaxAccelerationAngularDrive);
  }

  //Called when the scheduler first schedules the command
  @Override
  public void initialize() {}

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

    // square the inputs
    y = y * Math.abs(y);
    x = x * Math.abs(x);

    // apply dead-band
    if (Math.abs(x) < Constants.Swerve.kDeadband) {
      x = 0;
    }
    if (Math.abs(y) < Constants.Swerve.kDeadband) {
      y = 0;
    }
    theta = Math.abs(theta) > Constants.Swerve.kDeadband ? theta : 0.0;

    // apply slew rate limiter which also converts to m/s and rad.s
    x = xLimiter.calculate(x * Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond);
    y = yLimiter.calculate(y * Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond);
    //TODO: why doesn't theta get scaled as well??
    theta = turningLimiter.calculate(theta) * Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond;

    chassis.drive(x,y,theta);

    if (Constants.debugMode) {
      chassis.listener();
    }
  }

  /**
   * Called when the command is over.
   * Stops the chassis modules
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    chassis.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
