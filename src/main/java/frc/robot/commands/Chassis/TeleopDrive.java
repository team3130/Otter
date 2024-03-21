// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.SlewRateLimiterSpeeds;
import frc.robot.subsystems.Chassis;

/** A default command to drive in teleop based off the joysticks*/
public class TeleopDrive extends Command {
  private final Chassis chassis;
  private final PS5Controller controller;

  private final SlewRateLimiterSpeeds accelerationLimiter;
  private double y;
  private double x;
  private double theta;
  public TeleopDrive(Chassis chassis, PS5Controller PS5controller) {
    this.chassis = chassis;
    this.controller = PS5controller;

    // Use addRequirements() here to declare subsystem dependencies.
    m_requirements.add(chassis);

    accelerationLimiter = new SlewRateLimiterSpeeds(Constants.Swerve.kMaxAccelerationDrive, Constants.Swerve.kMaxDeccelerationDrive, Constants.Swerve.kMaxAccelerationAngularDrive, new ChassisSpeeds());
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
      // stack time (local var) > heap (instance var)
      double theta = 0d;
      double x = 0d;
      double y = 0d;
      double omega = 0d;

      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
          y = controller.getRawAxis(Constants.PS5.LST_AXS_LJOYSTICKX);
          x = controller.getRawAxis(Constants.PS5.LST_AXS_LJOYSTICKY);
      } else { // blue alliance
          y = -controller.getRawAxis(Constants.PS5.LST_AXS_LJOYSTICKX);
          x = -controller.getRawAxis(Constants.PS5.LST_AXS_LJOYSTICKY);
      }

      // theta the same for both alliances
      theta = -controller.getRawAxis(Constants.PS5.LST_AXS_RJOYSTICKX);

      /*
      // angle used for targeting
      omega = -controller.getRawAxis(Constants.PS5.LST_AXS_RJOYSTICKY);


      if (chassis.isTargetingSpeaker(omega, theta)) {
          chassis.resetTargetSpeakerController();
          theta = chassis.goToTargetPower();
      } else if (chassis.isTargetingBackClimb(omega, theta)) {
          chassis.resetTargetBackClimbController();
          theta = chassis.goToTargetPower();
      } else { // normal driving
          theta = Math.abs(theta) > Constants.Swerve.kDeadband ? theta : 0.0;
      }
      */

      // apply dead-band
      if (Math.abs(x) < Constants.Swerve.kDeadband) {
          x = 0;
      }
      if (Math.abs(y) < Constants.Swerve.kDeadband) {
          y = 0;
      }
      
      // square the inputs
      y = y * Math.abs(y);
      x = x * Math.abs(x);

    // convert joystick offsets to chassis speeds and apply slew rate limiter
      ChassisSpeeds newSpeeds = accelerationLimiter.calculate(new ChassisSpeeds(
              x     * Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond,
              y     * Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond,
              theta * Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond / (Constants.Swerve.trackWidth/2.0)
      ), new Translation2d(x,y));

      chassis.teleopDrive(newSpeeds);
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

  /**
   * @return false. Never is finished.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}