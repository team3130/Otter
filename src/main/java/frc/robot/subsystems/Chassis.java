// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.sensors.Camera;
import frc.robot.sensors.Navx;
import frc.robot.swerve.SwerveModule;

import java.util.Arrays;

/**
 * Chassis is the drivetrain subsystem of our bot. Our physical chassis is a swerve drive, 
 * so we use wpilib SwerveDriveKinematics and SwerveDrivePoseEstimator as opposed to Differential Drive objects
 */
public class Chassis extends SubsystemBase {
    private final SwerveDriveKinematics kinematics; // geometry of swerve modules
    private final SwerveDrivePoseEstimator odometry; // odometry object
    private final SwerveModule[] modules; // list of four swerve modules
    private final Navx navx = Navx.GetInstance(); // initialize Navx
    private boolean fieldRelative = true; // field relative or robot oriented drive
    private final Camera m_limelight;
    private double maxSpeedRead = 0; // updated periodically with the maximum speed that has been read on any of the swerve modules
    private final Field2d field; // sendable that gets put on shuffleboard with the auton trajectory and the robots current position
    private final GenericEntry n_fieldOrriented; // comp network table entry for whether field oriented drivetrain

    Rotation2d angleSetpoint = null;

    /**
     * Makes a chassis that starts at 0, 0, 0
     * @param limelight the limelight object that we can use for updating odometry
     */
    public Chassis(Camera limelight){
      this (new Pose2d(), new Rotation2d(), limelight);
    }

    /**
     * Makes a chassis with a starting position
     * @param startingPos the initial position to say that the robot is at
     * @param startingRotation the initial rotation of the bot
     * @param limelight the limelight object which is used for updating odometry
     */
    public Chassis(Pose2d startingPos, Rotation2d startingRotation, Camera limelight) {
        kinematics = new SwerveDriveKinematics(Constants.Swerve.moduleTranslations);

        modules = new SwerveModule[4];
        modules[Constants.Modules.leftFront] = new SwerveModule(Constants.Modules.leftFront);
        modules[Constants.Modules.leftBack] = new SwerveModule(Constants.Modules.leftBack);
        modules[Constants.Modules.rightFront] = new SwerveModule(Constants.Modules.rightFront);
        modules[Constants.Modules.rightBack] = new SwerveModule(Constants.Modules.rightBack);

        // odometry wrapper class that has functionality for cameras that report position with latency
        odometry = new SwerveDrivePoseEstimator(kinematics, startingRotation, generatePoses(), startingPos);

        m_limelight = limelight;

        field = new Field2d();
        Shuffleboard.getTab("Comp").add("field", field);
        n_fieldOrriented = Shuffleboard.getTab("Comp").add("field orriented", false).getEntry();

        AutoBuilder.configureHolonomic(
                this::getPose2d, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative,
                Constants.Swerve.holonomicPathFollowerConfig,
                RobotContainer.isFieldMirrored(),
                this // Reference to this subsystem to set requirements
        );
    }

    /**
     * Our main method to drive using three variables. Locked to field relative or robot oriented based off of fieldRelative
     * x is the velocity in the x dimension m/s
     * y is the velocity in the y dimension m/s
     * theta is the angular (holonomic) speed of the bot
     */
    public void drive(double x, double y, double theta) {
        drive(x, y, theta, getFieldRelative());
    }

    /**
     * drive(x, y, theta) with additional parameter for if robot is field relative or not
     * This method will drive the swerve modules based to x, y and theta vectors.
     */
    public void drive(double x, double y, double theta, boolean fieldRelative) {
        if (fieldRelative) {
            setModuleStates(kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, theta, getRotation2d())));
        } else {
            setModuleStates(kinematics.toSwerveModuleStates(new ChassisSpeeds(x, y, theta)));
        }
    }


    // If the PID controllers of the Swerve Modules are done, returing wheter the wheels are zeroed/PID controllers finished
    public boolean turnToAnglePIDIsFinished() {
        return modules[Constants.Modules.leftFront].PIDisDone() &&
            modules[Constants.Modules.leftBack].PIDisDone() &&
            modules[Constants.Modules.rightFront].PIDisDone() &&
            modules[Constants.Modules.rightBack].PIDisDone();
    }

    // Resets odometry: resets relative encoders to what the absolute encoders are, hard reset of odometry object
    // parameter pose is the pose2d to reset the odometry to
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(Navx.getRotation(), generatePoses(), pose);
    }

    // A listener to calculate what the max speed we read was
    public void listener() {
        for (SwerveModule module : modules) {
            if (maxSpeedRead < module.getDriveVelocity()) {
                maxSpeedRead = module.getDriveVelocity();
            }
        }
    }

    /**
     * Flip-flops between field relative and bot relative swerve drive
     */
    public void flipFieldRelative() {
        fieldRelative = !fieldRelative;
    }

    /**
     * Getter for if swerve drive is field relative or not
     * @return bool if field relative
     */
    public boolean getFieldRelative() {
        return fieldRelative;
    }

    /**
     * Returns the bots rotation according to Navx as a {@link Rotation2d}
     * @return the bot rotation
     */
    public Rotation2d getRotation2d(){
        return odometry.getEstimatedPosition().getRotation();
    }

    /**
     * Generates the positions of the swerve modules
     * @return the poses of each module
     */
    public SwerveModulePosition[] generatePoses() {
        return new SwerveModulePosition[] {
                modules[Constants.Modules.leftFront].getPosition(),
                modules[Constants.Modules.leftBack].getPosition(),
                modules[Constants.Modules.rightFront].getPosition(),
                modules[Constants.Modules.rightBack].getPosition()
        };
    }

    /**
     * periodic call to update odometry from encoders
     */
    public void updateOdometryFromSwerve() {
        odometry.update(Navx.getRotation(), generatePoses());
    }

    /**
     * subsystem looped call made by the scheduler.
     * Updates the odometry from swerve and April Tags.
     * Also updates and sendables we use during comp
     */
    @Override
    public void periodic() {
        n_fieldOrriented.setBoolean(fieldRelative);
        field.setRobotPose(odometry.getEstimatedPosition());
    }

    // Stops the devices connected to this subsystem
    public void stopModules(){
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    /**
     * Getter for geometry
     * @return the geometry of the swerve modules
     */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Sets the module states to desired states
     * @param desiredStates the states to set the modules to
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond);

        modules[Constants.Modules.leftFront].setDesiredState(desiredStates[Constants.Modules.leftFront]);
        modules[Constants.Modules.leftBack].setDesiredState(desiredStates[Constants.Modules.leftBack]);
        modules[Constants.Modules.rightFront].setDesiredState(desiredStates[Constants.Modules.rightFront]);
        modules[Constants.Modules.rightBack].setDesiredState(desiredStates[Constants.Modules.rightBack]);
    }

    /**
     * Spins the wheels to an angle
     * @param setpoint angle to spin the motors to
     */
    public void turnToAngle(double setpoint) {
        for (SwerveModule module : modules) {
            module.turnToAngle(setpoint);
        }
    }

    /**
     * The simulation periodic call
     */
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    /**
     * Command to reset the encoders
     */
    public void resetEncoders() {
        for (SwerveModule module : modules) {
            module.resetEncoders();
        }
    }

    /**
     * Sets field oriented to the provided boolean
     * @param fieldOriented to drive in field or robot orriented
     */
    public void setWhetherFieldOriented(boolean fieldOriented) {
        fieldRelative = fieldOriented;
    }

    /**
     * update the P values for the swerve module
     * @param pValue the new P value
     */
    public void updatePValuesFromSwerveModule(double pValue) {
        Arrays.stream(modules).forEach((SwerveModule modules) -> modules.updatePValue(pValue));
    }

    /**
     * update the D values for the swerve module
     * @param dValue the new D value
     */
    public void updateDValuesFromSwerveModule(double dValue) {
        Arrays.stream(modules).forEach((SwerveModule modules) -> modules.updateDValue(dValue));
    }

    /**
     * gets the kP values for each module
     * @return gets the kP value from the modules
     */
    public double getPValuesForSwerveModules() {
        return modules[0].getPValue();
    }

    /**
     * gets the kD values for each module
     * @return gets the kD value from the modules
     */
    public double getDValuesForSwerveModules() {
        return modules[0].getDValue();
    }

    /**
     * A vomit onto shuffleboard of the {@link SwerveModule} objects in Chassis
     * @param tab the tab to add the {@link SwerveModule} objects
     */
    public void exportSwerveModData(ShuffleboardTab tab) {
        tab.add(modules[0]);
        tab.add(modules[1]);
        tab.add(modules[2]);
        tab.add(modules[3]);
    }


    // returns the heading the NavX is reading, returning the rotation of the robot in degrees
    public double getHeading() {
        return Math.IEEEremainder(Navx.getAngle(), 360);
    }

    // return the x position from odometry
    private double getX() { return odometry.getEstimatedPosition().getX(); }

    // return the y position from odometry
    private double getY() { return odometry.getEstimatedPosition().getY(); }

    // the yaw from odometry
    private double getYaw() { return odometry.getEstimatedPosition().getRotation().getDegrees(); }

    // Gets the max speed field that we read thus far on this vm instance of rio
    public double getMaxSpeedRead() { return maxSpeedRead; }

    public String getOdometry() { return odometry.getEstimatedPosition().toString(); }

    /**
     * Initializes the data we send on shuffleboard
     * Calls the default init sendable for Subsystem Bases
     * @param builder sendable builder
     */
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Chassis");

        builder.addBooleanProperty("fieldRelative", this::getFieldRelative, this::setWhetherFieldOriented);
        builder.addDoubleProperty("Navx", this::getHeading, null);
        builder.addDoubleProperty("X position", this::getX, null);
        builder.addDoubleProperty("Y position", this::getY, null);
        builder.addDoubleProperty("rotation", this::getYaw, null);
        builder.addDoubleProperty("max speed read", this::getMaxSpeedRead, null);
        builder.addStringProperty("odometry pose2d", this::getOdometry, null);
    }

    /**
     * AUTONOMOUS
     */

    // returns the current position of the bot from odometry as a Pose2D
    public Pose2d getPose2d() {
        return odometry.getEstimatedPosition();
    }

    // method to reset the robot's odometry to the supplied pose
    public void resetPose(Pose2d newPose) {
        odometry.resetPosition(Navx.getRotation(), generatePoses(), newPose);
    }

    // ChassisSpeeds supplier in robot relative
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    // passes the x y omega ChassisSpeeds supplied by PathPlanner to driveAuton()
    public void driveRobotRelative(ChassisSpeeds speeds){
        driveAuton(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    // uses supplied ChassisSpeeds to drive autonomously in RobotRelative mode
    public void driveAuton(double x, double y, double theta) {
        setModuleStates(kinematics.toSwerveModuleStates(new ChassisSpeeds(x, y, theta)));
    }
}