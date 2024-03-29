// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SoftwareDeprecated.Testing;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.VelocityChassis;

/**
 * A command to automatically zero all odometry.
 */
public class DriveToPID extends InstantCommand {
  private final VelocityChassis m_chassis;

  private final SlewRateLimiter xLimiter;

  public DriveToPID(VelocityChassis subsystem) {
    m_chassis = subsystem;
    addRequirements(subsystem);

    xLimiter = new SlewRateLimiter(Constants.Swerve.kMaxAccelerationDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_chassis.resetOdometry(new Pose2d(0, 0, new Rotation2d()));

    m_chassis.updateDistancePIDVals();
  }

  @Override
  public void execute() {
    double x = m_chassis.goToDistancePower();


    // apply slew rate limiter which also converts to m/s and rad.s
    m_chassis.teleopDrive(x, 0, 0);
  }

  @Override
  public void end(boolean interrupted) {
    m_chassis.stopModules();
  }

  /**
   * @return false. Never is finished.
   */
  @Override
  public boolean isFinished() {
    return false;
  }

}
