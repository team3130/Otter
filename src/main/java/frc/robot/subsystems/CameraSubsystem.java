// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.Navx;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import static frc.robot.Constants.AprilTags.*;
import static frc.robot.Constants.maxRange;
import static frc.robot.Constants.minRange;

public class CameraSubsystem extends SubsystemBase {
  protected PhotonCamera camera = new PhotonCamera("cam");
  private static final ShuffleboardTab tab = Shuffleboard.getTab("PhotonCamera");
  AprilTagFieldLayout aprilTagFieldLayout;
  Pose2d targetPose = new Pose2d(16.58, 5.55, Rotation2d.fromRadians(0));
  Transform2d cameraToRobot = new Transform2d(3, 0, Rotation2d.fromRadians(0));
  private double previousPipelineTimestamp = 0;
  private int speakerTargetFiducialID;
  private int ampTargetFiducialID;
  private boolean isTryingToTarget = false;
  private int fiducialID = 0;
  private PIDController targetController;
  private  double targetP = 10d;
  private  double targetI = 0d;
  private  double targetD = 0d;

  private double range =
          PhotonUtils.calculateDistanceToTargetMeters(
                  CAMERA_HEIGHT_METERS,
                  TARGET_HEIGHT_METERS,
                  CAMERA_PITCH_RADIANS,
                  Units.degreesToRadians(getTarget().getPitch()));


  /**
   * Constructs a new Limelight object.
   * The limelight object will be full of null values if Constants.useAprilTags is false.
   */
  public CameraSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Vision");
    // Shuffleboard.getTab("Camerapls").add("target", target);
    tab.addBoolean("hasTarget", this::hasTarget);
    tab.addDouble("Target Yaw", this::getTargetYaw);
    tab.addInteger("fiducial ID", this::getFiducialID);

    // SuppliedValueWidget<Double> targetYaw = tab.addDouble("Target Yaw", this::getTargetYaw);
    // tab.addDouble("Target Yaw", this::getTargetYaw).withPosition(2, 0).withSize(6, 4)

    // hasTargetQuestion = Shuffleboard.getTab("Camerapls").add("hasTarget", hasTargets).getEntry();
    // hasTargetQuestion = Shuffleboard.getTab("Camerapls").add("targetYaw", targetYaw).getEntry();
    if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
      speakerTargetFiducialID = Constants.AprilTags.speakerTargetRedFiducialID;
      ampTargetFiducialID = Constants.AprilTags.ampTargetRedFiducialID;
    }
    else {
      speakerTargetFiducialID = Constants.AprilTags.speakerTargetBlueFiducialID;
      ampTargetFiducialID = Constants.AprilTags.ampTargetBlueFiducialID;
    }
    targetController = new PIDController(targetP, targetI, targetD);

  }
  public boolean targetControllerDone(){
    return targetController.atSetpoint();
  }
  public boolean isTryingToTarget(){
    return isTryingToTarget;
  }
  public void setTryingToTargetTrue(){
    isTryingToTarget=true;
  }

  //purpose of below system is to indicate to the driver and weapons officer if the robot is within the shooting range.
  //The boolean will mark true if we are in range and a April Tag is in sight, and it will be false if both conditions are not met.
  //TODO
  //look into integrating this into controller vibration for operator and driver.
  public boolean inShootingRange(){
    if (hasTarget() == true && maxRange >= range && minRange <= range){
      return true;
    }
    return false;
  }
  public void setTryingToTargetFalse(){
    isTryingToTarget=false;
  }

  public double getTargetP() {
    return targetP;
  }

  public double getTargetI() {
    return targetI;
  }

  public double getTargetD() {
    return targetD;
  }
  public void setTargetP(double newP){
    targetP = newP;
  }
  public void setTargetI(double newI){
    targetI = newI;
  }
  public void setTargetD(double newD){
    targetD = newD;
  }
  public boolean getIsTryingToTarget() {
    return isTryingToTarget;
  }

  public double goToTargetPower() {
    if(hasTarget()){
      return targetController.calculate(getTargetYaw(), 0);
    }
    else {
      return 0;
    }
  }
  public PhotonTrackedTarget getTarget() {
    if (!hasTarget()) {
      return null;
    } else {
      PhotonPipelineResult result = camera.getLatestResult();
      return result.getBestTarget();
    }
  }

  public void resetTargetController() {
    targetController.reset();
    targetController.setSetpoint(0d);
    targetController.enableContinuousInput(-2*Math.PI,2*Math.PI);
    targetController.setTolerance(Math.toRadians(1.0));
    targetController.setPID(targetP, targetI, targetD);
  }


  public Pose2d findRobotPose() {
    // cameraToRobot The position of the robot relative to the camera. If the camera was
    // mounted 3 inches behind the "origin" (usually physical center) of the robot, this would be
    // Transform2d(3 inches, 0 inches, 0 degrees).
    if (!hasTarget()) {
      return null;
    } else {
      PhotonPipelineResult result = camera.getLatestResult();
      double yaw = result.getBestTarget().getYaw();
      return PhotonUtils.estimateFieldToRobot(
              CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS,
              CAMERA_PITCH_RADIANS, Constants.AprilTags.kTargetPitch,
              Rotation2d.fromDegrees(yaw), Navx.getRotation(), targetPose, cameraToRobot);
    }
  }

  // AprilTagFieldLayout.getTagPose(getTarget().getFiducialId())

  public boolean hasTarget() {
    PhotonPipelineResult result = camera.getLatestResult();
    return result.hasTargets();
  }


  public double getTargetYaw() {
    if (!hasTarget()) {
      return -400.0;
    } else {
      PhotonPipelineResult result = camera.getLatestResult();
      if (result.getBestTarget() != null && result.getBestTarget().getPitch() != -400.0) {
        return -Math.toRadians(result.getBestTarget().getPitch());
      }
    }
    return -400d;
  }
  public double getTargetDegrees(){
    return Math.toDegrees(getTargetYaw());
  }




  public int getFiducialID(){
    return fiducialID;
  }

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Camera");
    builder.addBooleanProperty("hasTarget", this::hasTarget, null);
    builder.addDoubleProperty("targetYaw", this::getTargetDegrees, null);
    builder.addIntegerProperty("fiducial", this::getFiducialID, null);
    builder.addBooleanProperty("in Range", this::inShootingRange,null);

      builder.addDoubleProperty("target P", this::getTargetP, this::setTargetP);
      builder.addDoubleProperty("target I", this::getTargetI, this::setTargetI);
      builder.addDoubleProperty("target D", this::getTargetD, this::setTargetD);
      builder.addBooleanProperty("is targeting", this::getIsTryingToTarget, null);

  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    PhotonPipelineResult result = camera.getLatestResult();
    double resultTimestamp = result.getTimestampSeconds();
    if (result.hasTargets() && result.getBestTarget() != null) {
      previousPipelineTimestamp = resultTimestamp;
      PhotonTrackedTarget target = result.getBestTarget();
      fiducialID = result.getBestTarget().getFiducialId();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}