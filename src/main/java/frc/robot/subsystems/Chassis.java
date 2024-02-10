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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
    private PIDController robotAngleController;
    private double targetP = 10d;
    private double targetI = 0d;
    private double targetD = 0d;

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

        robotAngleController = new PIDController(targetP, targetI, targetD);
        robotAngleController.enableContinuousInput(-Math.PI, Math.PI); // wrap for circles
        robotAngleController.setTolerance(0.0025, 0.05); // at position tolerance

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
     * If the PID controllers of the {@link SwerveModule}'s are all done
     * @return whether the wheels are zereod/PID controllers are done
     */
    public void drive(double x, double y, double theta) {
        drive(x, y, theta, getFieldRelative());
    }
    public boolean getAutoConfig() {
        return AutoBuilder.isConfigured();
    }
    public boolean turnToAnglePIDIsDone() {
        return modules[Constants.Modules.leftFront].PIDisDone() &&
                modules[Constants.Modules.leftBack].PIDisDone() &&
                modules[Constants.Modules.rightFront].PIDisDone() &&
                modules[Constants.Modules.rightBack].PIDisDone();
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


    public Rotation2d getRotation2d(){
        return odometry.getEstimatedPosition().getRotation();
    }

    // periodic call to update odometry from encoders
    public void updateOdometryFromSwerve() {
        odometry.updateWithTime(Timer.getFPGATimestamp(), Navx.getRotation(), generatePoses());
    }

    // Resets odometry: resets relative encoders to what the absolute encoders are, hard reset of odometry object
    // parameter pose is the pose2d to reset the odometry to
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

    public void goToAnglePower(double angleSetpoint) {
        robotAngleController.calculate(getHeading(), angleSetpoint);
    }

    public boolean goToAnglePIDIsFinished() {
        return robotAngleController.atSetpoint();
    }

    public void shuffleboardUpdatePID() {
        robotAngleController.setPID(targetP, targetI, targetD);
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

    // set module states to desired states
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond);

        modules[Constants.Modules.leftFront].setDesiredState(desiredStates[Constants.Modules.leftFront]);
        modules[Constants.Modules.leftBack].setDesiredState(desiredStates[Constants.Modules.leftBack]);
        modules[Constants.Modules.rightFront].setDesiredState(desiredStates[Constants.Modules.rightFront]);
        modules[Constants.Modules.rightBack].setDesiredState(desiredStates[Constants.Modules.rightBack]);
    }


    // Spins the wheels to an angle
    public void turnToAngle(double setpoint) {
        for (SwerveModule module : modules) {
            module.turnToAngle(setpoint);
        }
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

    // returns the heading the NavX is reading, returning the rotation of the robot in radians
    public double getHeading() {
        return Math.toRadians(Math.IEEEremainder(Navx.getAngle(), 360));
    }

    public double getTargetP() { return targetP; }
    public double getTargetI() { return targetI; }
    public double getTargetD() { return targetD; }
    public void setTargetP(double p) { targetP = p; }
    public void setTargetI(double i) { targetI = i; }
    public void setTargetD(double d) { targetD = d; }


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

        builder.addDoubleProperty("Face Angle P", this::getTargetP, this::setTargetP);
        builder.addDoubleProperty("Face Angle I", this::getTargetI, this::setTargetI);
        builder.addDoubleProperty("Face Angle I", this::getTargetD, this::setTargetD);

        builder.addBooleanProperty("fieldRelative", this::getFieldRelative, this::setWhetherFieldOriented);
        builder.addDoubleProperty("Navx", this::getHeading, null);
        builder.addDoubleProperty("X position", this::getX, null);
        builder.addDoubleProperty("Y position", this::getY, null);
        builder.addDoubleProperty("rotation", this::getYaw, null);
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