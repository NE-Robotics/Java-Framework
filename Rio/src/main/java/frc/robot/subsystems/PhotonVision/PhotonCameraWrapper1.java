package frc.robot.subsystems.PhotonVision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.VisionConstants;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonCameraWrapper1 {
    private PhotonPoseEstimator photonPoseEstimator1;


    public PhotonCameraWrapper1() {
        final PhotonCamera photonCamera = new PhotonCamera(VisionConstants.k_cameraName1);
        try {
            // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
            AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            // Create pose estimator
            photonPoseEstimator1 =
                    new PhotonPoseEstimator(
                            fieldLayout, PoseStrategy.MULTI_TAG_PNP, photonCamera, VisionConstants.k_cameraToRobot1);
            photonPoseEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        } catch (IOException e) {
            // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
            // where the tags are.
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            photonPoseEstimator1 = null;
        }
    }

    /**
     * @param EstimatedRobotPose The current best guess at robot pose
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
     * the estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (photonPoseEstimator1 == null) {
            return Optional.empty();
        }
        photonPoseEstimator1.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator1.update();
    }
}
