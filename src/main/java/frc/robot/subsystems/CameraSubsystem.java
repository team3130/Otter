// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.Navx;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CameraSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public CameraSubsystem() {

  }
  private static final ShuffleboardTab tab = Shuffleboard.getTab("PhotonVision");
  AprilTagFieldLayout aprilTagFieldLayout;
  PhotonCamera camera = new PhotonCamera("photonvision");

  PhotonPipelineResult result = camera.getLatestResult();
  PhotonTrackedTarget target = result.getBestTarget();
  public boolean hasTargets = result.hasTargets();
  Pose2d targetPose = new Pose2d(0, 0, Rotation2d.fromRadians(0));
  Transform2d cameraToRobot = new Transform2d(3, 0, Rotation2d.fromRadians(0));

  /*
  cameraToRobot The position of the robot relative to the camera. If the camera was
  mounted 3 inches behind the "origin" (usually physical center) of the robot, this would be
  Transform2d(3 inches, 0 inches, 0 degrees).
  */
  Pose2d robotPose = PhotonUtils.estimateFieldToRobot(
          Constants.AprilTags.CAMERA_HEIGHT_METERS, Constants.AprilTags.TARGET_HEIGHT_METERS,
          Constants.AprilTags.CAMERA_PITCH_RADIANS, Constants.AprilTags.kTargetPitch,
          Rotation2d.fromDegrees(-target.getYaw()), Navx.getRotation(), targetPose, cameraToRobot);

  Rotation2d targetYaw = PhotonUtils.getYawToPose(robotPose, targetPose);

  // calculates Distance to Target
  double range =
          PhotonUtils.calculateDistanceToTargetMeters(
                  Constants.AprilTags.CAMERA_HEIGHT_METERS,
                  Constants.AprilTags.TARGET_HEIGHT_METERS,
                  Constants.AprilTags.CAMERA_PITCH_RADIANS,
                  Units.degreesToRadians(result.getBestTarget().getPitch()));

  // this is not how Camera shuffleboard works :(
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Camera");
    builder.addBooleanProperty("hasTarget", this::hasTarget, null);
    builder.addDoubleProperty("targetYaw", this::getTargetYaw, null);
  }

  private double getTargetYaw() {
    return targetYaw.getRadians();
  }

  public boolean hasTarget() {
    return hasTargets;
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
