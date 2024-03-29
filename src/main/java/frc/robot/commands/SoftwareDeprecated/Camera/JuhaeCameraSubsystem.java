// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.SoftwareDeprecated.Camera;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;


public class JuhaeCameraSubsystem extends SubsystemBase {
  protected PhotonCamera camera = new PhotonCamera("cam");
  private static final ShuffleboardTab tab = Shuffleboard.getTab("PhotonCamera");

  private PhotonTrackedTarget correctTarget =new PhotonTrackedTarget(0,0,0, 0, -1, new Transform3d(), new Transform3d(), 0, new ArrayList<>(), new ArrayList<>());
  private boolean hasTarget;
  double currentTimestamp = 0;
  private int fiducialID = 0;

  // TODO ambiguity;
  private int redCenterSpeakerTargetFiducialID = Constants.AprilTags.speakerTargetRedFiducialID;
  private int blueSpeakerTargetFiducialID = Constants.AprilTags.speakerTargetBlueFiducialID;
  private PIDController faceTargetController;
  private double faceTargetP = 10d;
  private double faceTargetI = 0d;
  private double faceTargetD = 0d;

  private Chassis chassis;


  private Translation2d robotToTarget;
  private Translation2d speakerTargetOdometry;
  private double scalarSetpointArcDistance = 9; // the distance of the arc to shoot at
  private PIDController snapToDistanceController;
  private double snapDistanceP = 10d;
  private double snapDistanceI = 0d;
  private double snapDistanceD = 0d;



  public JuhaeCameraSubsystem() {
    faceTargetController = new PIDController(faceTargetP, faceTargetI, faceTargetD);
    snapToDistanceController = new PIDController(snapDistanceP, snapDistanceI, snapDistanceD);
  }

  // vector of robot from field 0,0
  // STEP 1
  public Translation2d getLiveRobotPositionFromOdometry() {
    return new Translation2d(chassis.getOdometry().getEstimatedPosition().getX(),
            chassis.getOdometry().getEstimatedPosition().getY());
  }

  // vector from live camera of robot to target
  // STEP 2
  public Translation2d getLiveVisionRobotToTarget() {
    return new Translation2d(getDistanceToTarget(), chassis.getRotation2d());
  }

  // vector of target odometry stored from vision
  // STEP 3: storing
  public void setSpeakerTargetOdometry() {
    speakerTargetOdometry = getLiveVisionRobotToTarget().plus(getLiveRobotPositionFromOdometry());
  }

  // estimate vector of robot to target, without live view of a target after steps 1, 2, 3
  // STEP 1: without live hasTarget()
    // in initialize of snap PID controller
  public Translation2d getLastEstimatedRobotToTarget() {
    return speakerTargetOdometry.minus(getLiveRobotPositionFromOdometry());
  }

  // estimate vector of robot to setpoint distance in the arc
  // STEP 2: without live hasTarget()
  // in initialize of snap PID controller
  public Translation2d getVectorSetpointSnapToArc() {
    // returning the translation 2D of ( (last estimated robot to target distance - scalar arc distance) , last estimated robot to target angle )
    return new Translation2d(getLastEstimatedRobotToTarget().getDistance(new Translation2d(scalarSetpointArcDistance, getLastEstimatedRobotToTarget().getAngle())),
            getLastEstimatedRobotToTarget().getAngle());
  }

  // STEP 3: should be a scalar
  // should be initalize
  public double getSnapToTargetSetpoint() {
    return getVectorSetpointSnapToArc().getNorm(); // idk mikhail says trust
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

  public void resetSnapToDistanceController() {
    snapToDistanceController.setPID(snapDistanceP, snapDistanceI, snapDistanceD);
  }

  /*
  public double goToSnapDistancePower() {
    snapToDistanceController.calculate(getSnapToTargetSetpoint())
  }

   */


  /*
  SIMPLE PHOTON CAMERA GETTERS
   */
  public double getDistanceToTarget() {
    if (weAreUp()) {
      return PhotonUtils.calculateDistanceToTargetMeters(
              Constants.AprilTags.CAMERA_HEIGHT_METERS,
              Constants.AprilTags.TARGET_HEIGHT_METERS,
              Constants.AprilTags.CAMERA_PITCH_RADIANS,
              Units.degreesToRadians(correctTarget.getPitch())
      );
    } else {
      return -400d;
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
            correctTarget != null &&
            correctTarget.getYaw() != -400d &&
            correctTarget.getPitch() != -400d &&
            correctTarget.getFiducialId() != -1);
  }

  public void initSendable(SendableBuilder builder) {

  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    PhotonPipelineResult result = camera.getLatestResult();
    double resultTimestamp = result.getTimestampSeconds();
    hasTarget = result.hasTargets();
    int i =0;
    while (result.getTargets().size() > i && weAreUp() && resultTimestamp != currentTimestamp) {
      if (result.getTargets().get(i).getFiducialId() == Constants.AprilTags.speakerTargetRedFiducialID || result.getTargets().get(i).getFiducialId() == Constants.AprilTags.speakerTargetRedFiducialID) {
        fiducialID = result.getTargets().get(i).getFiducialId();
        setCurrentTag(result.getTargets().get(i));
        currentTimestamp = resultTimestamp;
        setSpeakerTargetOdometry();
      }
      i++;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}