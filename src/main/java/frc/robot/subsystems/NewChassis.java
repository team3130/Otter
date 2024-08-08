// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.Navx;

public class NewChassis extends SubsystemBase {
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator odometry;
  private final SwerveModule[] swerveModules;
  private final Navx navX;
  private boolean fieldOriented = true; //probably will never change
  private final Field2d field2d; //field that gets sent to shuffleboard with auton paths
  private final double maxVelocity = Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond; //for velocity control
  private final double maxAcceleration = Constants.Swerve.kMaxAccelerationDrive; //for velocity control
  private PIDController anglePIDController;
  private double kP = 0.0;
  private double kI = 0.0;
  private double kD = 0.0;
  private boolean isTargetingSpeaker = false;
  private boolean isTargetingAmp = false;

  public NewChassis() {

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
