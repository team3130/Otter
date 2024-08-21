// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensors.Navx;

public class NewChassis extends SubsystemBase {
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator odometry;
  private final SwerveModule[] swerveModules;
  private final Navx navX = Navx.GetInstance();
  private boolean fieldRelative = true; //probably will never change
  private final Field2d field; //field that gets sent to shuffleboard with auton paths
  private final double maxVelocity = Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond; //for velocity control
  private final double maxAcceleration = Constants.Swerve.kMaxAccelerationDrive; //for velocity control
  private PIDController anglePIDController;
  private double kP = 0.0;
  private double kI = 0.0;
  private double kD = 0.0;
  private PIDController targetController;
  private double XTargetV = 0.0; //idek what these do
  private double YTargetF = 0.0; //idek what these do

  public NewChassis(Pose2d startingPos, Rotation2d startingRotaion) {
    kinematics = new SwerveDriveKinematics(Constants.Swerve.moduleTranslations);

    swerveModules = new SwerveModule[4];
    swerveModules[Constants.SwerveModules.one] = new SwerveModule(Constants.SwerveModules.one);
    swerveModules[Constants.SwerveModules.two] = new SwerveModule(Constants.SwerveModules.two);
    swerveModules[Constants.SwerveModules.three] = new SwerveModule(Constants.SwerveModules.three);
    swerveModules[Constants.SwerveModules.four] = new SwerveModule(Constants.SwerveModules.four);

    //odometry wrapper class that can update position using camera with latency
    odometry = new SwerveDrivePoseEstimator(kinematics, startingRotaion, generatePoses(), startingPos);

    anglePIDController = new PIDController(kP,kI,kD);
    anglePIDController.enableContinuousInput(-Math.PI,Math.PI); //wheels use shortest distance
    anglePIDController.setTolerance(0.0025, 0.05);

    field = new Field2d();
    if (Constants.debugMode){
      SmartDashboard.putData("Field", field);
    }

    targetController = new PIDController(kP,kI,kD);
    targetController.enableContinuousInput(-Math.PI,Math.PI);
    targetController.setTolerance(Math.toRadians(1.0));
  }

  public NewChassis(){
    this (new Pose2d(),new Rotation2d());
  }


  public void teleopDrive(double x, double y, double theta, boolean fieldRelative){
    if (fieldRelative){
      setTeleopModuleStates(kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, theta, getRotation2d())));
    } else {
      setTeleopModuleStates(kinematics.toSwerveModuleStates(new ChassisSpeeds(x, y, theta)));
    }
  }

  public void teleopDrive(double x, double y, double theta){
    teleopDrive(x, y ,theta, fieldRelative);
  }

// returns positions of each swerve module
  public SwerveModulePosition[] generatePoses(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++){
      positions[i] = swerveModules[i].getPosition();
    }
    return positions;
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++){
      states[i] = swerveModules[i].getState();
    }
    return states;
  }

  //set modules to desired states
  public void setTeleopModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond);

    swerveModules[Constants.SwerveModules.one].setTeleopDesiredState(desiredStates[Constants.SwerveModules.one]);
    swerveModules[Constants.SwerveModules.two].setTeleopDesiredState(desiredStates[Constants.SwerveModules.two]);
    swerveModules[Constants.SwerveModules.three].setTeleopDesiredState(desiredStates[Constants.SwerveModules.three]);
    swerveModules[Constants.SwerveModules.four].setTeleopDesiredState(desiredStates[Constants.SwerveModules.four]);
  }
  public void setAutonModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond);

    swerveModules[Constants.SwerveModules.one].setAutonDesiredState(desiredStates[Constants.SwerveModules.one]);
    swerveModules[Constants.SwerveModules.two].setAutonDesiredState(desiredStates[Constants.SwerveModules.two]);
    swerveModules[Constants.SwerveModules.three].setAutonDesiredState(desiredStates[Constants.SwerveModules.three]);
    swerveModules[Constants.SwerveModules.four].setAutonDesiredState(desiredStates[Constants.SwerveModules.four]);
  }


  public void exportSwerveModData(ShuffleboardTab tab){
    tab.add(swerveModules[0]);
    tab.add(swerveModules[1]);
    tab.add(swerveModules[2]);
    tab.add(swerveModules[3]);
  }
  public void flipFieldRelative(){
    fieldRelative = !fieldRelative;
  }
  public void resetNavXHeading(){
    Navx.resetNavX();
  }
  public void resetEncoders(){
    for (SwerveModule module : swerveModules){
      module.resetEncoders();
    }
  }
  public void resetOdometry(Pose2d newPose){
    resetEncoders();
    resetNavXHeading();
    odometry.resetPosition(Navx.getRotation(), generatePoses(), newPose);
  }
  public void updateOdometryFromSwerve(){
    odometry.updateWithTime(Timer.getFPGATimestamp(), Navx.getRotation(), generatePoses());
  }
  public void stopModules(){
    for (SwerveModule module : swerveModules){
      module.stop();
    }
  }
  public double goToTargetPower(){
    return targetController.calculate(getRotation2d().getRadians());
  }


  //getters
  public Pose2d getPose2d(){
    return odometry.getEstimatedPosition();
  }
  public Rotation2d getRotation2d(){
    return odometry.getEstimatedPosition().getRotation();
  }
  public boolean getFieldRelative(){
    return fieldRelative;
  }
  public double getkP(){return kP;}
  public double getkI(){return kI;}
  public double getkD(){return kD;}

  //setters
  public void setPose(Pose2d newPose){
    odometry.resetPosition(Navx.getRotation(), generatePoses(), newPose);
  }
  public void setFieldRelative(boolean fieldOriented){
    fieldRelative = fieldOriented;
  }
  public void setkP(double value){kP = value;}
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Constants.debugMode){
      field.setRobotPose(odometry.getEstimatedPosition());
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
