// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;


public class CameraSubsystem extends SubsystemBase {
  protected PhotonCamera camera = new PhotonCamera("3130cam");
  private static final ShuffleboardTab tab = Shuffleboard.getTab("PhotonCamera");

  private PhotonTrackedTarget correctTarget =new PhotonTrackedTarget(0,0,0, 0, -1,
          new Transform3d(), new Transform3d(), 0, new ArrayList<>(), new ArrayList<>());
  private boolean hasTarget;
  double currentTimestamp = 0;
  private int fiducialID = 0;

  // TODO ambiguity;
  private PIDController faceTargetController;
  private double faceTargetP = 0.24;
  private double faceTargetI = 0d;
  private double faceTargetD = 0d;
  private double goalDistanceMeters = 3.505;

  private Chassis chassis;


  private Translation2d robotToTarget;
  private Translation2d speakerTargetOdometry;
  private double scalarSetpointArcDistance = 9; // the distance of the arc to shoot at

  public CameraSubsystem() {
    faceTargetController = new PIDController(faceTargetP, faceTargetI, faceTargetD);

    faceTargetController.setSetpoint(0d);
    faceTargetController.enableContinuousInput(-2*Math.PI,2*Math.PI);
    faceTargetController.setTolerance(Math.toRadians(1.0));
  }


  public void resetFaceTargetController() {
    faceTargetController.reset();
    faceTargetController.setPID(faceTargetP, faceTargetP, faceTargetP);
  }

  public boolean atFaceTargetSetpoint() {
    return faceTargetController.atSetpoint();
  }

  public double goToFaceTargetPower() {
    if (weAreUp()) {
      return faceTargetController.calculate(getTargetYaw(), 0);
    } else {
      return 0;
    }
  }

  /*
  SIMPLE PHOTON CAMERA GETTERS
   */
  public double getDistanceToTarget() {
    if (weAreUp()) {
      return PhotonUtils.calculateDistanceToTargetMeters(
              Constants.AprilTags.CAMERA_HEIGHT_METERS,
              Constants.AprilTags.TARGET_HEIGHT_METERS,
              Constants.AprilTags.CAMERA_PITCH_RADIANS,
              Units.degreesToRadians(-correctTarget.getPitch())
      );
    } else {
      return -400d;
    }
  }

  public boolean atShootingDistance() {
    if ((Math.abs(getDistanceToTarget() - goalDistanceMeters) < 0.2) && weAreUp()) {
      return true;
    } else {
      return false;
    }
  }

  public double getTargetYaw() {
    if (weAreUp()) {
      return Math.toRadians(correctTarget.getYaw());
    } else {
      return -400d;
    }
  }

  public boolean hasTarget() {
    return camera.hasTargets();
  }

  public void setCurrentTag(PhotonTrackedTarget target){
    correctTarget = target;
  }

  public boolean weAreUp(){
    return (camera != null &&
            camera.getLatestResult() != null &&
            camera.getLatestResult().hasTargets() &&
            correctTarget != null);
  }

  public double getFaceTargetP() { return faceTargetP;}
  public double getFaceTargetI() { return faceTargetI;}
  public double getFaceTargetD() { return faceTargetD;}
  public void setFaceTargetP(double p) { faceTargetP = p;}
  public void setFaceTargetI(double i) { faceTargetI = i;}
  public void setFaceTargetD(double d) { faceTargetD = d;}
  //public double getGoalDist(){
    //return getGoalDist();
  //}
  public void setGoalDist(double dist){
     goalDistanceMeters =dist;
  }
  public double getFiducialID(){
    return fiducialID;
  }

  // This method will be called once per scheduler run

  @Override
  public void periodic() {
    PhotonPipelineResult result = camera.getLatestResult();
    int i = 0;
    while (result.getTargets().size() > i && result.getTargets() != null && hasTarget() && result.getTargets().get(i).getFiducialId() != -1) {
      if (result.getTargets().get(i).getFiducialId() == 4 || result.getTargets().get(i).getFiducialId() == 7) {
        fiducialID = result.getTargets().get(i).getFiducialId();
        setCurrentTag(result.getTargets().get(i));
      }
      i++;
    }
  }


  public void initSendable(SendableBuilder builder) {
    if (Constants.debugMode) {
      builder.setSmartDashboardType("Camera");
      builder.addDoubleProperty("Face Target P", this::getFaceTargetP, this::setFaceTargetP);
      builder.addDoubleProperty("Target Yaw", this::getTargetYaw, null);
      builder.addDoubleProperty("Target Dist", this::getDistanceToTarget, null);
      //builder.addDoubleProperty("Ideal Dist", this::getGoalDist, this::setGoalDist);

      builder.addDoubleProperty("Face Target I", this::getFaceTargetI, this::setFaceTargetI);
      builder.addDoubleProperty("Face Target D", this::getFaceTargetD, this::setFaceTargetD);
      builder.addBooleanProperty("At Distance", this::atShootingDistance, null);
      builder.addDoubleProperty("Fidcuacial ID", this::getFiducialID, null);
    }
  }

}