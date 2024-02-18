// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.sensors.Navx;
import frc.robot.swerve.SwerveModule;

/**
 * Chassis is the drivetrain subsystem of our bot. Our physical chassis is a swerve drive,
 * so we use wpilib SwerveDriveKinematics and SwerveDrivePoseEstimator as opposed to Differential Drive objects
 */
public class Chassis extends SubsystemBase {
    private final SwerveDriveKinematics kinematics; // geometry of swerve modules
    private final SwerveDrivePoseEstimator odometry; // odometry object
    private final SwerveModule[] modules; // list of four swerve modules
    private double maxSpeedRead = 0; // updated periodically with the maximum speed that has been read on any of the swerve modules
    private final Field2d field; // sendable that gets put on shuffleboard with the auton trajectory and the robots current position

    /**
     * Makes a chassis that starts at 0, 0, 0
     */
    public Chassis(){
        this (new Pose2d(), new Rotation2d());
    }

    /**
     * Makes a chassis with a starting position
     * @param startingPos the initial position to say that the robot is at
     * @param startingRotation the initial rotation of the bot
     */
    public Chassis(Pose2d startingPos, Rotation2d startingRotation) {
        kinematics = new SwerveDriveKinematics(Constants.Swerve.moduleTranslations);

        modules = new SwerveModule[Constants.Swerve.moduleTranslations.length];
        for (int i = 0; i < modules.length; i++) {
            modules[i] = new SwerveModule(i);
        }

        // odometry wrapper class that has functionality for cameras that report position with latency
        odometry = new SwerveDrivePoseEstimator(kinematics, startingRotation, getSwervePositions(), startingPos);


        field = new Field2d();
        SmartDashboard.putData("Field", field);

        AutoBuilder.configureHolonomic(
            this::getPose2d, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds,   // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::drive,                    // ChassisSpeeds consumer. MUST BE ROBOT RELATIVE
            Constants.Swerve.holonomicPathFollowerConfig,
            RobotContainer.isFieldMirrored(),
            this // Reference to this subsystem to set requirements
        );
    }

    /**
     * Our main method to drive using ChassisSpeed object. Always robot oriented
     * For field relative driving convert the ChassisSpeed object before calling this method:
     *      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, chassis.getRotation2d())
     */
    public void drive(ChassisSpeeds speeds) {
        setModuleStates(kinematics.toSwerveModuleStates(speeds));
    }

    public Rotation2d getRotation2d(){
        return odometry.getEstimatedPosition().getRotation();
    }

    // periodic call to update odometry from encoders
    public void updateOdometryFromSwerve() {
        odometry.update(Navx.getRotation(), getSwervePositions());
    }

    // Resets odometry: resets relative encoders to what the absolute encoders are,
    // hard reset of odometry object
    // parameter pose is the pose2d to reset the odometry to
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        Navx.resetNavX();
        odometry.resetPosition(Navx.getRotation(), getSwervePositions(), pose);
    }

    // Generates an array of swerve module positions, returns the array
    public SwerveModulePosition[] getSwervePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    // Generates an array of swerve module states, returns the array
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    // set module states to desired states
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(desiredStates[i]);
        }
    }

    // Command to reset the encoders
    public void resetEncoders() {
        for (SwerveModule module : modules) {
            module.resetEncoders();
        }
    }

    /**
     * subsystem looped call made by the scheduler.
     * Updates the odometry from swerve and April Tags.
     * Also updates and sendables we use during comp
     */
    @Override
    public void periodic() {
        field.setRobotPose(odometry.getEstimatedPosition());
    }

    /**
     * A vomit onto shuffleboard of the {@link SwerveModule} objects in Chassis
     * @param tab the tab to add the {@link SwerveModule} objects
     */
    public void exportSwerveModData(ShuffleboardTab tab) {
        for (SwerveModule module : modules) {
            tab.add(module);
        }
    }

    // Gets the max speed field that we read thus far on this vm instance of rio
    public double getMaxSpeedRead() { return maxSpeedRead; }

    public String getOdometry() { return getPose2d().toString(); }

    /**
     * Initializes the data we send on shuffleboard
     * Calls the default init sendable for Subsystem Bases
     * @param builder sendable builder
     */
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Chassis");
        builder.addDoubleProperty("max speed read", this::getMaxSpeedRead, null);
        builder.addStringProperty("odometry pose2d", this::getOdometry, null);
    }

    /**
     * AUTONOMOUS PATHPLANNER METHODS
     */

    // returns the current position of the bot from odometry as a Pose2D
    public Pose2d getPose2d() {
        return odometry.getEstimatedPosition();
    }

    // method to reset the robot's odometry to the supplied pose
    public void resetPose(Pose2d newPose) {
        odometry.resetPosition(Navx.getRotation(), getSwervePositions(), newPose);
    }

    // ChassisSpeeds supplier in robot relative
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }
}