// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.Chassis;

/** A default command to drive in teleop based off the joysticks*/
public class TeleopDrive extends Command {
  private final Chassis chassis;
  private final PS5Controller controller;
  private final CameraSubsystem camera;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private double y;
  private double x;
  private double theta;
  public TeleopDrive(Chassis chassis, PS5Controller PS5controller, CameraSubsystem camera) {
    this.chassis = chassis;
    this.controller = PS5controller;
    this.camera = camera;

    // Use addRequirements() here to declare subsystem dependencies.
    m_requirements.add(chassis);

    xLimiter = new SlewRateLimiter(Constants.Swerve.kMaxAccelerationDrive);
    yLimiter = new SlewRateLimiter(Constants.Swerve.kMaxAccelerationDrive);
    turningLimiter = new SlewRateLimiter(Constants.Swerve.kMaxAccelerationAngularDrive);
  }

  /**
   * Called when the scheduler first schedules the command
   */
  @Override
  public void initialize() {
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
          y = controller.getRawAxis(Constants.PS5.AXS_LJOYSTICKX);
          x = controller.getRawAxis(Constants.PS5.AXS_LJOYSTICKY);
      } else { // blue alliance
          y = -controller.getRawAxis(Constants.PS5.AXS_LJOYSTICKX);
          x = -controller.getRawAxis(Constants.PS5.AXS_LJOYSTICKY);
      }

      // theta the same for both alliances
      theta = -controller.getRawAxis(Constants.PS5.AXS_RJOYSTICK_X);

      // angle used for targeting
      omega = -controller.getRawAxis(Constants.PS5.AXS_RJOYSTICK_Y);

      if (chassis.isTargetingSpeaker(omega, theta)) {
          camera.resetFaceTargetController();
          theta = camera.goToFaceTargetPower();
          /*
          chassis.resetTargetSpeakerController();
          theta = chassis.goToTargetPower();

           */
      } else if (chassis.isTargetingAmp(omega, theta)){
          chassis.resetTargetAmpController();
          theta = chassis.goToTargetPower();
      }
      else if (chassis.isTargetingPodium(omega, theta)) {
          chassis.resetTargetPodiumController();
          theta = chassis.goToTargetPower();
      } else { // normal driving
          theta = Math.abs(theta) > Constants.Swerve.kDeadband ? theta : 0.0;
          theta = turningLimiter.calculate(theta) * Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond;
      }


      // square the inputs
      y = y * Math.abs(y);
      x = x * Math.abs(x);

      // apply dead-band
      if ( ( (x * x) + (y * y) ) <= (Constants.Swerve.kDeadband * Constants.Swerve.kDeadband)) {
          x = 0;
          y = 0;}
      // apply dead-band


      // apply slew rate limiter which also converts to m/s and rad.s
      x = xLimiter.calculate(x * Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond);
      y = yLimiter.calculate(y * Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond);

      chassis.teleopDrive(x, y, theta); //uses either driving or targeting inputs for theta
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