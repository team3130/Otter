// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.VelocityChassis;

/**
 * A command to automatically zero all odometry.
 */
public class ResetOdometryForward extends InstantCommand {
  private final Chassis m_chassis;

  public ResetOdometryForward(Chassis subsystem) {
    m_chassis = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red && DriverStation.getAlliance().isPresent()) {
      m_chassis.resetOdometry(new Pose2d(0, 0, new Rotation2d(Math.toRadians(180))));
    } else { */
      m_chassis.resetOdometry(new Pose2d(0, 0, new Rotation2d()));

  }
}
