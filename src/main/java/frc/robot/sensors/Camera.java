package frc.robot.sensors;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.GenericEntry;
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

public class Camera {

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
     */
    public Camera() {
        if (Constants.AprilTags.useAprilTags) {
            camera = new PhotonCamera("OV5647");
            try {
                aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
                if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
                    aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
                } // TODO: check if blue is default or if we need to add Blueberry
            } catch (IOException e) {
                DriverStation.reportError("error loading field position file", false);
            }

            // filter = new VisionMedianFilter(Constants.AprilTags.kMedianFilterWindowSize);

            /*
            if (Constants.debugMode) {
                SendableRegistry.add(filter, "vision filter");
                SmartDashboard.putData(filter);
            }
            */

            n_yaw = Shuffleboard.getTab("PhotonVision").add("Yaw", 0).getEntry();
            n_pitch = Shuffleboard.getTab("PhotonVision").add("Pitch", 0).getEntry();
        }   
    }

    /**
     * Calculates the position of the bot relative to an april tag.
     * That calculation is then given to {@link VisionMedianFilter}. // giorgia and juhar this is inverse and incorrect !
     * The command will return null if there is no new information or if there are no targets in frame.
     * This will add all the targets that are currently visible to the {@link VisionMedianFilter}.
     * If {@link Constants#useAprilTags} is false, this will return null.
     *
     * @return the filtered camera position
     */
    public OdoPosition calculate() {
        if (Constants.useAprilTags == false || camera.isConnected() == false) {
            return null;
        }
        // the most recent result as read by the camera
        PhotonPipelineResult result = camera.getLatestResult();

        // if there is no new results or if there are no targets on the screen
        if (result.getTimestampSeconds() == lastReadTime || !result.hasTargets()) {
            return null;
        }

        // increment the amount of successful updates we have read
        successfulUpdates++;
        lastReadTime = result.getTimestampSeconds();

        // default value for what we will return
        OdoPosition best = null;

        // for each target that is currently on the screen
        for (PhotonTrackedTarget target : result.getTargets()) { // what if this is null
            // x is forward, y is left, z is up
            Transform3d bestCameraToTarget = target.getBestCameraToTarget(); // juhar thinks this is the most accurate transformation done

            // if target is further away from robot in this case, (0, 0, 0), then move on to next target
            if (bestCameraToTarget.getTranslation().getDistance(new Translation3d(0 ,0, 0)) > Constants.AprilTagTrustDistance) {
                continue;
            }

            // the matrix transformation for the camera to the center of the bot
            Transform3d cameraToCenterOfBot = new Transform3d(
                    new Translation3d(Camera.xPos, Camera.yPos, Camera.zPos),
                    new Rotation3d(Camera.roll, Camera.pitch, Camera.yaw));

            Optional<Pose3d> pose3d = aprilTagFieldLayout.getTagPose(target.getFiducialId());

            if (pose3d.isEmpty()) {
                continue;
            }

            // the position of the bot relative to the april tag
            Pose3d position = PhotonUtils.estimateFieldToRobotAprilTag(
                    bestCameraToTarget,
                    pose3d.get(),
                    cameraToCenterOfBot);

            /* updates the best value that we will return on the last iteration,
              also passes the read position into the {@link VisionMedianFilter)
             */
            best = filter.getOdoPose(
                    new OdoPosition(position.toPose2d(), result.getTimestampSeconds()));
        }
        // returns the last filtered value that we checked in the above for loop
        return best;
    }

}

