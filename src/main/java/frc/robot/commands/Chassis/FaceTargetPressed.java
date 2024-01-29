// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.Chassis;

/** An example command that uses an example subsystem. */
public class FaceTargetPressed extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CameraSubsystem camera;

  private final Chassis chassis;
  private final XboxController xboxController;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  public FaceTargetPressed(Chassis chassis, XboxController xboxController, CameraSubsystem camera) {
    this.chassis = chassis;
    this.camera = camera;
    this.xboxController = xboxController;

    // Use addRequirements() here to declare subsystem dependencies.
    m_requirements.add(chassis);
    m_requirements.add(camera);

    xLimiter = new SlewRateLimiter(Constants.Swerve.kMaxAccelerationDrive);
    yLimiter = new SlewRateLimiter(Constants.Swerve.kMaxAccelerationDrive);
    turningLimiter = new SlewRateLimiter(Constants.Swerve.kMaxAccelerationAngularDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    camera.resetTargetController();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double theta = 0.0;
    double y = xboxController.getRawAxis(Constants.Buttons.LST_AXS_LJOYSTICKX); // left stick y-axis (y-axis is inverted)
    double x = xboxController.getRawAxis(Constants.Buttons.LST_AXS_LJOYSTICKY); // left stick x-axis

    if (camera.isTryingToTarget()){
      theta = camera.goToTargetPower();

    } else {
      theta = -xboxController.getRawAxis(Constants.Buttons.LST_AXS_RJOYSTICKX); // right stick x-axis
      theta = Math.abs(theta) > Constants.Swerve.kDeadband ? theta : 0.0;
      theta = turningLimiter.calculate(theta) * Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond;
    }
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

    // apply slew rate limiter which also converts to m/s and rad.s
    x = xLimiter.calculate(x * Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond);
    y = yLimiter.calculate(y * Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond);

    chassis.drive(x,y,theta);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
