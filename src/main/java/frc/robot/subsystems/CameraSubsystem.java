// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.DistanceCalcs;
import frc.robot.sensors.Navx;
import frc.robot.sensors.LinearInterp;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;

public class CameraSubsystem extends SubsystemBase {
  protected PhotonCamera camera = new PhotonCamera("cam");
  private static final ShuffleboardTab tab = Shuffleboard.getTab("PhotonCamera");
  AprilTagFieldLayout aprilTagFieldLayout;
  private DistanceCalcs distanceCalcs = new DistanceCalcs();



  private double cameraHeight =  0.965; //meters
  private double speakerTagHeight = 1.3; //meters
  private double cameraPitch = 0; //radians
  private double Angle1LowBound = 0d;
  private double Angle1HighBound = 2d;
  private double Angle2HighBound = 4d;
  private double Angle3HighBound = 6d;

  double currentTimestamp = 0;



  Pose2d targetPose = new Pose2d(16.58, 5.55, Rotation2d.fromRadians(0));
  Transform2d cameraToRobot = new Transform2d(3, 0, Rotation2d.fromRadians(0));
  private PhotonTrackedTarget CorrectTarget =new PhotonTrackedTarget(0,0,0, 0, -1, new Transform3d(), new Transform3d(), 0, new ArrayList<>(), new ArrayList<>());
  private double previousPipelineTimestamp = 0;
  private int redSpeakerTargetFiducialID;
  private int blueSpeakerTargetFiducialID;

  private int ampTargetFiducialID;
  private boolean isTryingToTarget = false;
  private int fiducialID = 0;
  private PIDController targetController;
  private  double targetP = 10d;
  private  double targetI = 0d;
  private  double targetD = 0d;

  private double XtargetV = 0d;
  private double YtargetF = 0d;


  /**
   * Constructs a new Limelight object.
   * The limelight object will be full of null values if Constants.useAprilTags is false.
   */
  public CameraSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Vision");
    // Shuffleboard.getTab("Camerapls").add("target", target);
    tab.addBoolean("hasTarget", this::hasTarget);
    tab.addDouble("Target Yaw", this::getTargetYaw);

    // SuppliedValueWidget<Double> targetYaw = tab.addDouble("Target Yaw", this::getTargetYaw);
    // tab.addDouble("Target Yaw", this::getTargetYaw).withPosition(2, 0).withSize(6, 4)

    // hasTargetQuestion = Shuffleboard.getTab("Camerapls").add("hasTarget", hasTargets).getEntry();
    // hasTargetQuestion = Shuffleboard.getTab("Camerapls").add("targetYaw", targetYaw).getEntry();
        redSpeakerTargetFiducialID = Constants.AprilTags.speakerTargetRedFiducialID;

        blueSpeakerTargetFiducialID = Constants.AprilTags.speakerTargetBlueFiducialID;


      targetController = new PIDController(targetP, targetI, targetD);

    }

  public boolean facingTarget(){
    return targetController.atSetpoint();
  }
  public boolean readyToShoot(int shooterPose){
    return (shooterPose==desiredAngle() && facingTarget());
  }
  public boolean shouldBeAngle1(){
    return (getREALdistToTarget() <= Angle1HighBound);
  }
  public boolean shouldBeAngle2(){
    return (getREALdistToTarget() <= Angle2HighBound);
  }
  public boolean shouldBeAngle3(){
    return (getREALdistToTarget() <= Angle3HighBound);
  }

  public int desiredAngle(){
    if(getREALdistToTarget() <= Angle1HighBound){
      return 1;
    }else if (getREALdistToTarget() <= Angle2HighBound){
      return 2;
    }else if(getREALdistToTarget() <= Angle3HighBound){
      return 3;
    }else {return 0;}
  }
  public boolean isTryingToTarget(){
    return isTryingToTarget;
  }
  public void setTryingToTargetTrue(){
    isTryingToTarget=true;
  }
  public void setTryingToTargetFalse(){
    isTryingToTarget=false;
  }
  public void setXTargetV(double newXF){
    XtargetV = newXF ;
  }
  public void setYTargetV(double newYF){
    YtargetF = newYF ;
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

  public double getXTargetV() {
    return XtargetV;
  }
  public double getYTargetV() {
    return YtargetF;
  }
  public int getFiducialID() {
    return fiducialID;
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
              Constants.AprilTags.CAMERA_HEIGHT_METERS, Constants.AprilTags.TARGET_HEIGHT_METERS,
              Constants.AprilTags.CAMERA_PITCH_RADIANS, Constants.AprilTags.kTargetPitch,
              Rotation2d.fromDegrees(yaw), Navx.getRotation(), targetPose, cameraToRobot);
    }
  }

  // AprilTagFieldLayout.getTagPose(getTarget().getFiducialId())

  public boolean hasTarget() {
    return camera.hasTargets();
  }
  public boolean weAreUp(){
    return ( camera != null &&
            camera.getLatestResult() != null &&
            camera.getLatestResult().hasTargets() &&
            CorrectTarget != null &&
            CorrectTarget.getYaw() != -400d &&
            CorrectTarget.getPitch() != -400d &&
            CorrectTarget.getFiducialId() != -1);
  }
  public int getCorrectTargetID(){
    return fiducialID;
  }
  public double getTargetYaw() {
    if (weAreUp()) {
        return Math.toRadians(CorrectTarget.getYaw());
    } else {
    return -400d;
    }
  }
  public double getTargetPitch() {
    if (!hasTarget()) {
      return -400.0;
    } else {
      if (weAreUp()) {
        return Math.toRadians(CorrectTarget.getPitch());
      }
    }
    return -400d;
  }
  public double getTargetDegrees() {
    return Math.toDegrees(getTargetYaw());
  }

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Camera");
    builder.addBooleanProperty("hasTarget", this::hasTarget, null);
    builder.addDoubleProperty("targetYaw", this::getTargetDegrees, null);
    builder.addIntegerProperty("fiducial", this::getCorrectTargetID, null);

    builder.addDoubleProperty("target P", this::getTargetP, this::setTargetP);
    builder.addDoubleProperty("target I", this::getTargetI, this::setTargetI);
    builder.addDoubleProperty("target D", this::getTargetD, this::setTargetD);
    builder.addBooleanProperty("is targeting", this::isTryingToTarget, null);
    builder.addDoubleProperty("target YF", this::getYTargetV, this::setYTargetV);
    builder.addDoubleProperty("target XF", this::getXTargetV, this::setXTargetV);
    builder.addDoubleProperty("target dist", this::getFakeTargetDist, null);
    builder.addDoubleProperty("REAL dist", this::getREALdistToTarget, null);



  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    PhotonPipelineResult result = camera.getLatestResult();
    double resultTimestamp = result.getTimestampSeconds();
    int i = 0;
    while (result.getTargets().size() > i && weAreUp() && resultTimestamp != currentTimestamp) {
      if (result.getTargets().get(i).getFiducialId() == 4 || result.getTargets().get(i).getFiducialId() == 7) {
        fiducialID = result.getTargets().get(i).getFiducialId();
        setCurrentTag(result.getTargets().get(i));
        currentTimestamp = resultTimestamp;
      }
      i++;
    }
  }
  public void setCurrentTag(PhotonTrackedTarget target){
    CorrectTarget = target;
  }
  public double getFakeTargetDist() {
    if (weAreUp()) {
      return PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, speakerTagHeight, cameraPitch, getTargetPitch());
    }
    else return 0d;
  }
  public double getREALdistToTarget() {
    return distanceCalcs.getREALDistanceToTarget(getFakeTargetDist());
  }



  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}