// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
  AprilTagFieldLayout aprilTagFieldLayout;
  Pose2d targetPose = new Pose2d(16.58, 5.55, Rotation2d.fromRadians(0));
  Transform2d cameraToRobot = new Transform2d(3, 0, Rotation2d.fromRadians(0));
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

  private double XtargetV = 0;
  private double YtargetF = 0;
  private Chassis chassis;


  /**
   * Constructs a new Limelight object.
   * The limelight object will be full of null values if Constants.useAprilTags is false.
   */
  public CameraSubsystem(Chassis chassiss) {
    chassis = chassiss;

    // SuppliedValueWidget<Double> targetYaw = tab.addDouble("Target Yaw", this::getTargetYaw);
    // tab.addDouble("Target Yaw", this::getTargetYaw).withPosition(2, 0).withSize(6, 4)

    // hasTargetQuestion = Shuffleboard.getTab("Camerapls").add("hasTarget", hasTargets).getEntry();
    // hasTargetQuestion = Shuffleboard.getTab("Camerapls").add("targetYaw", targetYaw).getEntry();


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
  public void setTryingToTargetFalse(){
    isTryingToTarget=false;
  }

  public void toggleIsTryingToTarget() {
    isTryingToTarget = !isTryingToTarget;
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
      return targetController.calculate(chassis.getRotation2d().getRadians(), Math.PI);
  }


  public void resetTargetController() {
    targetController.reset();
    targetController.setSetpoint(Math.PI);
    targetController.enableContinuousInput(-Math.PI,Math.PI);
    targetController.setTolerance(Math.toRadians(1.0));
    targetController.setPID(targetP, targetI, targetD);
  }



  // AprilTagFieldLayout.getTagPose(getTarget().getFiducialId())

  public int getCorrectTargetID(){
    return fiducialID;
  }


  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("target P", this::getTargetP, this::setTargetP);
    builder.addDoubleProperty("target I", this::getTargetI, this::setTargetI);
    builder.addDoubleProperty("target D", this::getTargetD, this::setTargetD);
    builder.addBooleanProperty("is targeting", this::isTryingToTarget, null);
    builder.addDoubleProperty("target F", this::getXTargetV, this::setXTargetV);
    builder.addDoubleProperty("target YF", this::getYTargetV, this::setYTargetV);
    builder.addDoubleProperty("target XF", this::getXTargetV, this::setXTargetV);



  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}