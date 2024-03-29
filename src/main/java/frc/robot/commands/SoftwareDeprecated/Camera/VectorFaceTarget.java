// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SoftwareDeprecated.Camera;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.Chassis;

/** A default command to drive in teleop based off the joysticks*/
public class VectorFaceTarget extends Command {
  private final Chassis chassis;
  private final PS5Controller controller;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private final CameraSubsystem camera;

  public VectorFaceTarget(Chassis chassis, PS5Controller PS5controller, CameraSubsystem camera) {
    this.chassis = chassis;
    this.controller = PS5controller;
    this.camera = camera;
    m_requirements.add(chassis);
    m_requirements.add(camera);

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
    double thetaJoystick = 0d;
    double xJoystick = 0d;
    double yJoystick = 0d;
    double omegaJoystick = 0d;

    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      yJoystick = controller.getRawAxis(Constants.PS5.AXS_LJOYSTICKX);
      xJoystick = controller.getRawAxis(Constants.PS5.AXS_LJOYSTICKY);
    } else { // blue alliance
      yJoystick = -controller.getRawAxis(Constants.PS5.AXS_LJOYSTICKX);
      xJoystick = -controller.getRawAxis(Constants.PS5.AXS_LJOYSTICKY);
    }

    // theta the same for both alliances
    thetaJoystick = -controller.getRawAxis(Constants.PS5.AXS_RJOYSTICK_X);

    // angle used for targeting
    omegaJoystick = -controller.getRawAxis(Constants.PS5.AXS_RJOYSTICK_Y);



    /*
    if (camera.isTryingToTarget() && (thetaJoystick != 0d)){
      thetaJoystick = (xJoystick * camera.getXTargetV()) + (camera.goToTargetPower()) + (-yJoystick * camera.getYTargetV());
    } else {
      thetaJoystick = thetaJoystick;
      thetaJoystick = Math.abs(thetaJoystick) > Constants.Swerve.kDeadband ? thetaJoystick : 0.0;
      thetaJoystick = turningLimiter.calculate(thetaJoystick) * Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond;
    }

     */
    // square the inputs
    yJoystick = yJoystick * Math.abs(yJoystick);
    xJoystick = xJoystick * Math.abs(xJoystick);


    // square the inputs
    yJoystick = yJoystick * Math.abs(yJoystick);
    xJoystick = xJoystick * Math.abs(xJoystick);

    // apply dead-band
    if (Math.abs(xJoystick) < Constants.Swerve.kDeadband) {
      xJoystick = 0;
    }
    if (Math.abs(yJoystick) < Constants.Swerve.kDeadband) {
      yJoystick = 0;
    }

    // apply slew rate limiter which also converts to m/s and rad.s
    xJoystick = xLimiter.calculate(xJoystick * Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond);
    yJoystick = yLimiter.calculate(yJoystick * Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond);

    chassis.teleopDrive(xJoystick, yJoystick, thetaJoystick); //uses either driving or targeting inputs for theta
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