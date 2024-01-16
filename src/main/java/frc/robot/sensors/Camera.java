package frc.robot.sensors;

import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.Optional;

import static edu.wpi.first.units.Units.Meters;

public class Camera {
    private static final ShuffleboardTab tab = Shuffleboard.getTab("PhotonVision");
    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonCamera camera = new PhotonCamera("photonvision");
    GenericEntry hasTargetQuestion;

    PhotonPipelineResult result = camera.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    public boolean hasTargets = result.hasTargets();
    Pose2d targetPose = new Pose2d(0, 0, Rotation2d.fromRadians(0));
    Transform2d cameraToRobot = new Transform2d(3, 0, Rotation2d.fromRadians(0));

    public Camera() {
        Shuffleboard.getTab("Camerapls").add("target", target);
        hasTargetQuestion = Shuffleboard.getTab("Camerapls").add("hasTarget", hasTargets).getEntry();
        hasTargetQuestion = Shuffleboard.getTab("Camerapls").add("targetYaw", targetYaw).getEntry();
    }

    /*
    cameraToRobot The position of the robot relative to the camera. If the camera was
    mounted 3 inches behind the "origin" (usually physical center) of the robot, this would be
    Transform2d(3 inches, 0 inches, 0 degrees).
    */
    Pose2d robotPose = PhotonUtils.estimateFieldToRobot(
            Constants.AprilTags.CAMERA_HEIGHT_METERS, Constants.AprilTags.TARGET_HEIGHT_METERS,
            Constants.AprilTags.CAMERA_PITCH_RADIANS, Constants.AprilTags.kTargetPitch,
            Rotation2d.fromDegrees(-target.getYaw()), Navx.getRotation(), targetPose, cameraToRobot);

    Rotation2d targetYaw = PhotonUtils.getYawToPose(robotPose, targetPose);

    // calculates Distance to Target
    double range =
            PhotonUtils.calculateDistanceToTargetMeters(
                    Constants.AprilTags.CAMERA_HEIGHT_METERS,
                    Constants.AprilTags.TARGET_HEIGHT_METERS,
                    Constants.AprilTags.CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(result.getBestTarget().getPitch()));

    // this is not how Camera shuffleboard works :(
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Camera");
        builder.addBooleanProperty("hasTarget", this::hasTarget, null);
        builder.addDoubleProperty("targetYaw", this::getTargetYaw, null);
    }

    private double getTargetYaw() {
        return targetYaw.getRadians();
    }

    public boolean hasTarget() {
        return hasTargets;
    }
}


    /*
    protected PhotonCamera camera;
    private static final ShuffleboardTab tab = Shuffleboard.getTab("PhotonCamera");
    AprilTagFieldLayout aprilTagFieldLayout;
    int successfulUpdates = 0;

    GenericEntry n_yaw;
    GenericEntry n_pitch;

    protected double lastReadTime = 0;

    /**
     * Constructs a new Limelight object.
     * The limelight object will be full of null values if Constants.useAprilTags is false.

    public Camera() {
        if (Constants.AprilTags.useAprilTags) {
            camera = new PhotonCamera("OV5647");
            try {
                aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
                // TODO: is this equals real?
                if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
                    aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
                } // TODO: check if blue is default or if we need to add Blueberry
            } catch (IOException e) {
                DriverStation.reportError("error loading field position file", false);
            }

            // filter = new VisionMedianFilter(Constants.AprilTags.kMedianFilterWindowSize);


            if (Constants.debugMode) {
                SendableRegistry.add(filter, "vision filter");
                SmartDashboard.putData(filter);
            }


            n_yaw = Shuffleboard.getTab("PhotonVision").add("Yaw", 0).getEntry();
            n_pitch = Shuffleboard.getTab("PhotonVision").add("Pitch", 0).getEntry();
        }   
    }
     */

