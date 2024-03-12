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
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private final CameraSubsystem camera;
  private double y;
  private double x;
  private double theta;
  public TeleopDrive(Chassis chassis, PS5Controller PS5controller, CameraSubsystem camera) {
    this.chassis = chassis;
    this.controller = PS5controller;
    this.camera = camera;

    // Use addRequirements() here to declare subsystem dependencies.
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
    double robotTheta = 0d;
    double robotX = 0d;
    double robotY = 0d;
    double robotTargetAngle = 0d;

    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      robotY = controller.getRawAxis(Constants.PS5.LST_AXS_LJOYSTICKX);
      robotX = controller.getRawAxis(Constants.PS5.LST_AXS_LJOYSTICKY);
    } else { // blue alliance
      robotY = -controller.getRawAxis(Constants.PS5.LST_AXS_LJOYSTICKX);
      robotX = -controller.getRawAxis(Constants.PS5.LST_AXS_LJOYSTICKY);
    }
    robotTheta = -controller.getRawAxis(Constants.PS5.LST_AXS_RJOYSTICKX); // theta the same for both alliances

    robotTargetAngle = -controller.getRawAxis(Constants.PS5.LST_AXS_RJOYSTICKY); // angle used for targeting

    // press up on right joystick is vector targeting faceTarget
    if (robotTheta != 0) {
      camera/chassis.simpleFaceTarget();
      chassis.vectorFaceAprilTag();
      robotTheta = -camera.targetController.calculate(chassis.getRotation2d().getRadians(), Math.toRadians(chassis.getTheta()));
    } else if (chassis.isTargetingSpeaker) {

    } else { // normal driving
      theta = Math.abs(theta) > Constants.Swerve.kDeadband ? theta : 0.0;
      theta = turningLimiter.calculate(theta) * Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond;
    }

    // square the inputs
    y = y * Math.abs(y);
    x = x * Math.abs(x);

    // apply dead-band
    if (Math.abs(x) < Constants.Swerve.kDeadband) { x = 0; }
    if (Math.abs(y) < Constants.Swerve.kDeadband) { y = 0; }

    // apply slew rate limiter which also converts to m/s and rad/s
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