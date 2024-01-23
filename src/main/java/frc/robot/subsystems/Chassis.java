// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
    private final Navx Gyro = Navx.GetInstance(); // initialize Navx
    private boolean fieldRelative = true; // field relative or robot oriented drive
    private final CameraSubsystem cameraSubsystem;
    private double maxSpeedRead = 0; // updated periodically with the maximum speed that has been read on any of the swerve modules
    private final Field2d field; // sendable that gets put on shuffleboard with the auton trajectory and the robots current position
    private final GenericEntry n_fieldOrriented; // comp network table entry for whether field oriented drivetrain
    private PIDController targetController;
    //private TrapezoidProfile.Constraints targetConstraints;
    private  double targetP = 10d;
    private  double targetI = 0d;
    private  double targetD = 0d;
    private double targetMaxVelo = Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond; //TODO real
    private double targetMaxAcc = Constants.Swerve.kMaxAccelerationDrive; //TODO real

    private boolean isTryingToTarget = false;



    /**
     * Makes a chassis that starts at 0, 0, 0
     * the limelight object that we can use for updating odometry
     */
    public Chassis(CameraSubsystem cameraSubsystem){
        this (new Pose2d(), new Rotation2d(), cameraSubsystem);
    }

    /**
     * Makes a chassis with a starting position
     * @param startingPos the initial position to say that the robot is at
     * @param startingRotation the initial rotation of the bot
     * the limelight object which is used for updating odometry
     */
    public Chassis(Pose2d startingPos, Rotation2d startingRotation, CameraSubsystem cameraSubsystem1) {
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
        Shuffleboard.getTab("Comp").add("field", field);
        n_fieldOrriented = Shuffleboard.getTab("Comp").add("field orriented", false).getEntry();

        targetController = new PIDController(targetP, targetI, targetD);

    }

    public void resetTargetController() {
        targetController.reset();
        targetController.setSetpoint(0d);
        targetController.enableContinuousInput(-2*Math.PI,2*Math.PI);
        targetController.setTolerance(Math.toRadians(1.0));
        targetController.setPID(targetP, targetI, targetD);    }
    public double goToTargetPower() {
        if(cameraSubsystem.hasTarget()){
            return targetController.calculate(cameraSubsystem.getTargetYaw(), 0);
        }
        else {
            return 0;
        }
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



    /**
     * If the PID controllers of the {@link SwerveModule}'s are all done
     * @return whether the wheels are zereod/PID controllers are done
     */
    public boolean turnToAnglePIDIsDone() {
        return modules[Constants.Modules.leftFront].PIDisDone() &&
                modules[Constants.Modules.leftBack].PIDisDone() &&
                modules[Constants.Modules.rightFront].PIDisDone() &&
                modules[Constants.Modules.rightBack].PIDisDone();
    }

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

    // Zeros the Navx's heading
    public void zeroHeading(){
        Navx.resetNavX();
    }

    /**
     * Returns the heading that navx reads
     * @return the rotation of the bot in degrees
     */
    public double getHeading() {
        return Math.IEEEremainder(Navx.getAngle(), 360);
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
     * Also provides a timestamp that the update occurred
     */
    public void updateOdometryFromSwerve() {
        odometry.updateWithTime(Timer.getFPGATimestamp(), Navx.getRotation(), generatePoses());
    }

    // Update odometry with swerve drive
    public void updateOdometery() {
        updateOdometryFromSwerve();
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
        modules[Constants.Modules.leftFront].stop();
        modules[Constants.Modules.leftBack].stop();
        modules[Constants.Modules.rightFront].stop();
        modules[Constants.Modules.rightBack].stop();
    }

    /**
     * Getter for geometry
     * @return the geometry of the swerve modules
     */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
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
     * Returns the current position of the bot as a {@link Pose2d}
     * @return position according to odometry
     */
    public Pose2d getPose2d() {
        return odometry.getEstimatedPosition();
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
     * Sets the bot to be field relative
     */
    public void setFieldRelative() {
        fieldRelative = true;
    }

    /**
     * Sets the bot to robot oriented
     */
    public void setRobotOriented() {
        fieldRelative = false;
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
     * @return the x position from odometry
     */
    private double getX() {
        return odometry.getEstimatedPosition().getX();
    }

    /**
     * @return the y position from odometry
     */
    private double getY() {
        return odometry.getEstimatedPosition().getY();
    }

    /**
     * @return the yaw from odometry
     */
    private double getYaw() {
        return odometry.getEstimatedPosition().getRotation().getDegrees();
    }

    /**
     * A vomit onto shuffleboard of the {@link SwerveModule} objects in Chassis
     * @param tab the tab to add the {@link SwerveModule} objects
     */
    public void shuffleboardVom(ShuffleboardTab tab) {
        tab.add(modules[0]);
        tab.add(modules[1]);
        tab.add(modules[2]);
        tab.add(modules[3]);
    }

    /**
     * Initializes the data we send on shuffleboard
     * Calls the default init sendable for Subsystem Bases
     * @param builder sendable builder
     */
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Chassis");

        // add field relative
        builder.addBooleanProperty("fieldRelative", this::getFieldRelative, this::setWhetherFieldOriented);
        builder.addDoubleProperty("Navx", this::getHeading, null);
        builder.addDoubleProperty("X position", this::getX, null);
        builder.addDoubleProperty("Y position", this::getY, null);
        builder.addDoubleProperty("rotation", this::getYaw, null);
        builder.addDoubleProperty("max speed read", this::getMaxSpeedRead, null);
        builder.addDoubleProperty("target P", this::getTargetP, this::setTargetP);
        builder.addDoubleProperty("target I", this::getTargetI, this::setTargetI);
        builder.addDoubleProperty("target D", this::getTargetD, this::setTargetD);
        builder.addBooleanProperty("is targeting", this::getIsTryingToTarget, null);
    }

    /**
     * A listener to calculate what the max speed we read was
     */
    public void listener() {
        for (SwerveModule module : modules) {
            if (maxSpeedRead < module.getDriveVelocity()) {
                maxSpeedRead = module.getDriveVelocity();
            }
        }
    }

    /**
     * Gets the max speed field
     * @return the max speed that we read thus far on this vm instance of rio
     */
    public double getMaxSpeedRead() {
        return maxSpeedRead;
    }

    /**
     * The same as {@link #drive(double, double, double)} except you pass in if you are field relative or not.
     * This method will drive the swerve modules based to x, y and theta vectors.
     * @param x velocity in the x dimension m/s
     * @param y velocity in the y dimension m/s
     * @param theta the angular (holonomic) speed to drive the swerve modules at
     * @param fieldRelative whether to use
     */
    public void drive(double x, double y, double theta, boolean fieldRelative) {
        if (fieldRelative) {
            setModuleStates(kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, theta, getRotation2d())));
        }
        else {
            setModuleStates(kinematics.toSwerveModuleStates(new ChassisSpeeds(x, y, theta)));
        }
    }

    /**
     * Our main method to drive using three variables. Locked to field relative or robot oriented based off of {@link #fieldRelative}.
     * @param x the velocity in the x dimension m/s
     * @param y the velocity in the y dimension m/s
     * @param theta the angular (holonomic) speed of the bot
     */
    public void drive(double x, double y, double theta) {
        drive(x, y, theta, getFieldRelative());
    }

}