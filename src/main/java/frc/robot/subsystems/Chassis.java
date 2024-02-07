// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Autos;
import frc.robot.RobotContainer;
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
    private double maxSpeedRead = 0; // updated periodically with the maximum speed that has been read on any of the swerve modules
    private final Field2d field; // sendable that gets put on shuffleboard with the auton trajectory and the robots current position
    private final GenericEntry n_fieldOrriented; // comp network table entry for whether field oriented drivetrain
    private double targetMaxVelo = Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond; //TODO real
    private double targetMaxAcc = Constants.Swerve.kMaxAccelerationDrive; //TODO real
    private Pose2d initialPosition;
    private double initialAprilTagDistance = 0;
    private double initialAprilTagAngle;
    private Translation2d initialAprilTagVector;
    private Translation2d originToAprilTagVector;
    private double theta = 0.0;
    private boolean isFaceTargetting = false;
    Rotation2d angleSetpoint = null;

    /**
     * Makes a chassis that starts at 0, 0, 0
     * the limelight object that we can use for updating odometry
     */
    public Chassis(){
        this (new Pose2d(), new Rotation2d());
    }

    /**
     * Makes a chassis with a starting position
     * @param startingPos the initial position to say that the robot is at
     * @param startingRotation the initial rotation of the bot
     * the limelight object which is used for updating odometry
     */
    public Chassis(Pose2d startingPos, Rotation2d startingRotation) {
        kinematics = new SwerveDriveKinematics(Constants.Swerve.moduleTranslations);


        modules = new SwerveModule[4];
        modules[Constants.Modules.leftFront] = new SwerveModule(Constants.Modules.leftFront);
        modules[Constants.Modules.leftBack] = new SwerveModule(Constants.Modules.leftBack);
        modules[Constants.Modules.rightFront] = new SwerveModule(Constants.Modules.rightFront);
        modules[Constants.Modules.rightBack] = new SwerveModule(Constants.Modules.rightBack);


        // odometry wrapper class that has functionality for cameras that report position with latency
        odometry = new SwerveDrivePoseEstimator(kinematics, startingRotation, generatePoses(), startingPos);


        cameraSubsystem = cameraSubsystem1;


        field = new Field2d();
        SmartDashboard.putData("Field", field);
        n_fieldOrriented = Shuffleboard.getTab("Chassis").add("field orriented", false).getEntry();

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

    // Flip-flops between field relative and bot relative swerve drive
    public void flipFieldRelative() {
        fieldRelative = !fieldRelative;
    }

    // Getter for if swerve drive is field relative or not, bool if field relative
    public boolean getFieldRelative() {
        return fieldRelative;
    }

    // Zeros the Navx's heading
    public void zeroHeading(){
        Navx.resetNavX();
    }
    
    // sets field oriented (field or robot oriented) to the provided boolean
    public void setWhetherFieldOriented(boolean fieldOriented) {
        fieldRelative = fieldOriented;
    }

    /**
    * If the PID controllers of the {@link SwerveModule}'s are all done
    * @return whether the wheels are zereod/PID controllers are done
    */
     /** If the PID controllers of the {@link SwerveModule}'s are all done
     * @return whether the wheels are zereod/PID controllers are done
     */
    public boolean turnToAnglePIDIsDone() {
        return modules[Constants.Modules.leftFront].PIDisDone() &&
                modules[Constants.Modules.leftBack].PIDisDone() &&
                modules[Constants.Modules.rightFront].PIDisDone() &&
                modules[Constants.Modules.rightBack].PIDisDone();
    }
    // returns the bots rotation according to NavX
    public Rotation2d getRotation2d(){
        return odometry.getEstimatedPosition().getRotation();
    }

    // Resets odometry: resets relative encoders to what the absolute encoders are, hard reset of odometry object
    // parameter pose is the pose2d to reset the odometry to
    /**
     * Resets odometry
     * <p>Resets navx</p>
     * <p>Resets relative encoders to be what the absolute encoders are</p>
     * <p>Hard reset of the odometry object</p>
     * @param pose the position to reset odometry to
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        Navx.resetNavX();
        odometry.resetPosition(Navx.getRotation(), generatePoses(), pose);
    }

    // If the PID controllers of the Swerve Modules are done, returning whether the wheels are zeroed/PID controllers finished
    public boolean turnToAnglePIDIsFinished() {
        return modules[Constants.Modules.leftFront].PIDisDone() &&
                modules[Constants.Modules.leftBack].PIDisDone() &&
                modules[Constants.Modules.rightFront].PIDisDone() &&
                modules[Constants.Modules.rightBack].PIDisDone();
    }

    // Generates the position of the swerve modules, retuning the position
    public SwerveModulePosition[] generatePoses() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }
    /**
     * Getter for if swerve drive is field relative or not
     * @return bool if field relative
     */

    // Zeros the Navx's heading
    public void zeroHeading() {
        Navx.resetNavX();
    }
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the heading that navx reads
     * @return the rotation of the bot in degrees
     */
    // set module states to desired states
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond);

        modules[Constants.Modules.leftFront].setDesiredState(desiredStates[Constants.Modules.leftFront]);
        modules[Constants.Modules.leftBack].setDesiredState(desiredStates[Constants.Modules.leftBack]);
        modules[Constants.Modules.rightFront].setDesiredState(desiredStates[Constants.Modules.rightFront]);
        modules[Constants.Modules.rightBack].setDesiredState(desiredStates[Constants.Modules.rightBack]);
    }

    /**
     * Returns the bots rotation according to Navx as a {@link Rotation2d}
     * @return the bot rotation
     */
    // A listener to calculate what the max speed we read was
    public void listener() {
        for (SwerveModule module : modules) {
            if (maxSpeedRead < module.getDriveVelocity()) {
                maxSpeedRead = module.getDriveVelocity();
            }
        }
    }

    // Spins the wheels to an angle
    public void turnToAngle(double setpoint) {
        for (SwerveModule module : modules) {
            module.turnToAngle(setpoint);
        }
    }

    /**
     * periodic call to update odometry from encoders
     * Also provides a timestamp that the update occurred
     */
    public void updateOdometryFromSwerve() {
        odometry.updateWithTime(Timer.getFPGATimestamp(), Navx.getRotation(), generatePoses());
    }
    // Stops the devices connected to this subsystem
    public void stopModules(){
        for (SwerveModule module : modules) {
            module.stop();
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
        n_fieldOrriented.setBoolean(fieldRelative);
        field.setRobotPose(odometry.getEstimatedPosition());
    }

    /**
     * Getter for geometry
     * @return the geometry of the swerve modules
     */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }
    // ChassisSpeeds supplier in robot relative
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * The simulation periodic call
     */
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
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

    // update the P values for the swerve module
    public void updatePValuesFromSwerveModule(double pValue) {
        Arrays.stream(modules).forEach((SwerveModule modules) -> modules.updatePValue(pValue));
    }

    // update the D values for the swerve module
    public void updateDValuesFromSwerveModule(double dValue) {
        Arrays.stream(modules).forEach((SwerveModule modules) -> modules.updateDValue(dValue));
    }

    // gets the kP values for each module
    public double getPValuesForSwerveModules() {
        return modules[0].getPValue();
    }

    // gets the kD values for each module
    public double getDValuesForSwerveModules() {
        return modules[0].getDValue();
    }

    // returns the heading the NavX is reading, returning the rotation of the robot in degrees
    public double getHeading() {
        return Math.IEEEremainder(Navx.getAngle(), 360);
    }

    // return the x position from odometry
    private double getX() { return odometry.getEstimatedPosition().getX(); }
    // return the y position from odometry
    private double getY() { return odometry.getEstimatedPosition().getY(); }

    /**
     * @return the yaw from odometry
     */
    public double getInitialAprilTagDistance() { return initialAprilTagDistance; }
    public Translation2d getOriginToAprilTagVector() {return originToAprilTagVector;}
    public void setFaceTargetting(boolean newIsFaceTargetting){
        isFaceTargetting = newIsFaceTargetting;
    }
    public boolean getFaceTargetting(){
        return isFaceTargetting;
    }

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
        builder.addDoubleProperty("Target Distance", this::getInitialAprilTagDistance, null);
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
        odometry.resetPosition(Navx.getRotation(), generatePoses(), newPose);
    }

    /*This method generates the angle needed to turn to face a specific target without using a camera
       - needs to be paired with Giorgia's face target code or needs to start with the camera facing the target
     */
    public double getAngleToFaceTarget(Translation2d originToAprilTagVector) {
        // the vector from the position that we are at currently to the april tag (target)
        Translation2d currentPositionToAprilTagVector = getPose2d().getTranslation().minus(originToAprilTagVector);
        // the angle that we need to turn to from our current holonomic at our current position to face target (setpoint)
        theta = Math.atan2(currentPositionToAprilTagVector.getY(), currentPositionToAprilTagVector.getX()) - Math.PI;
        return theta;
    }

    // getter for the initial position where we use Giorgia's face target
    public Pose2d getInitialPosition() {
        return getPose2d();
    }

    // gets the information like distance to april tag and angle to make vectors (uses FaceTarget info)
    public void prepareForFaceTarget(){
        initialPosition = getInitialPosition();
        initialAprilTagDistance = cameraSubsystem.getTargetDistance();
        initialAprilTagAngle = initialPosition.getRotation().getRadians(); // can maybe use yaw later
        initialAprilTagVector = new Translation2d(initialAprilTagDistance * Math.sin(Math.PI - initialAprilTagAngle), initialAprilTagDistance * Math.cos(Math.PI - initialAprilTagAngle));
        originToAprilTagVector = initialPosition.getTranslation().plus(initialAprilTagVector);
    }
    /**
     * The same as {@link #drive(double, double, double)} except you pass in if you are field relative or not.
     * This method will drive the swerve modules based to x, y and theta vectors.
     *
     * @param //x             velocity in the x dimension m/s
     * @param //y             velocity in the y dimension m/s
     * @param //theta         the angular (holonomic) speed to drive the swerve modules at
     * @param //fieldRelative whether to use
     * @return
     */


    // passes the x y omega ChassisSpeeds supplied by PathPlanner to driveAuton()
    public void driveRobotRelative(ChassisSpeeds speeds){
        driveAuton(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    // uses supplied ChassisSpeeds to drive autonomously in RobotRelative mode
    public void driveAuton(double x, double y, double theta) {
        setModuleStates(kinematics.toSwerveModuleStates(new ChassisSpeeds(x, y, theta)));
    }
}
