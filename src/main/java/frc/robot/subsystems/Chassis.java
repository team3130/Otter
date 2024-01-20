// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
                //this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                (speeds) -> drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, true),
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        // TODO: change constants below
                        new PIDConstants(1.4, 0.05, 0.015), // Translation PID constants
                        new PIDConstants(1.4, 0.05, 0.015), // Rotation PID constants
                        3, // Max module speed, in m/s
                        0.35, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    // I genuinely dk
    // also written to be runVelocity
    /*
    public void driveRobotRelative(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, Constants.Swerve.kPhysicalMaxSpeedMetersPerSecond);

        // Send setpoints to modules
        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            // The module returns the optimized state, useful for logging
            optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
        }
    }
    */

    /** Runs the module with the specified setpoint state. Returns the optimized state.
    public SwerveModuleState runSetpoint(SwerveModuleState state) {
        // Optimize state based on current angle
        // Controllers run in "periodic" when the setpoint is not null
        var optimizedState = SwerveModuleState.optimize(state, getRotation2d());

        // Update setpoints, controllers run in "periodic"
        angleSetpoint = optimizedState.angle;
        speedSetpoint = optimizedState.speedMetersPerSecond;

        return optimizedState;
    }
     */

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

    // method to reset the robot's odometry to the given pose
    public void resetPose(Pose2d newPose) {
        odometry.resetPosition(getRotation2d(), generatePoses(), newPose);
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

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
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
    public void exportSwerveModData(ShuffleboardTab tab) {
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