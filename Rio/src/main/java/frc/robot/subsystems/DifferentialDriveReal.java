package frc.robot.subsystems;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.subsystems.PhotonVision.PhotonCameraWrapper1;
import frc.robot.subsystems.PhotonVision.PhotonCameraWrapper2;
import frc.robot.supers.DifferentialDriveSuper;

import org.photonvision.EstimatedRobotPose;

import utilities.vision.AprilTags;

import java.util.Optional;

public class DifferentialDriveReal extends DifferentialDriveSuper {
    // TODO swap to single class
    private final PhotonCameraWrapper1 photonPoseEstimator1 = new PhotonCameraWrapper1();
    private final PhotonCameraWrapper2 photonPoseEstimator2 = new PhotonCameraWrapper2();
    public DifferentialDriveReal() {
        configureMotors();
        resetEncoders();
        setupDrive();
    }

    @Override
    public void periodic() {
        updatePoseEstimator();
        renderFieldData();
    }
    public void resetEncoders() {
        leftFront.getEncoder().setPosition(0);
        rightFront.getEncoder().setPosition(0);
    }
    @Override
    public double getLeftEncoderDistanceMeters() {
        return Constants.EncoderConstants.realPositionToDistance(leftFront.getEncoder().getPosition());
    }

    @Override
    public double getRightEncoderDistanceMeters() {
        return Constants.EncoderConstants.realPositionToDistance(rightFront.getEncoder().getPosition());
    }

    private void setupDrive() {
        poseEstimator = new DifferentialDrivePoseEstimator(
                driveKinematics,
                driveOdometry.getPoseMeters().getRotation(),
                getLeftEncoderDistanceMeters(),
                getRightEncoderDistanceMeters(),
                driveOdometry.getPoseMeters()
        );
    }

    private void updatePoseEstimator() {
        poseEstimator.update(
                driveOdometry.getPoseMeters().getRotation(),
                getLeftEncoderDistanceMeters(),
                getRightEncoderDistanceMeters()
        );
        Optional<EstimatedRobotPose> result1 = photonPoseEstimator1.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
        Optional<EstimatedRobotPose> result2 = photonPoseEstimator2.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
        double poseX = poseEstimator.getEstimatedPosition().getX();
        Rotation2d poseHeading = poseEstimator.getEstimatedPosition().getRotation();
        if(result1.isPresent()) {
            EstimatedRobotPose estimatedPose = result1.get();
            poseEstimator.setVisionMeasurementStdDevs(AprilTags.calculateVisionUncertainty(poseX, poseHeading,
                    Constants.VisionConstants.k_cameraToRobot1.getRotation().toRotation2d(),
                    Constants.VisionConstants.k_cameraName1));
            poseEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
        }
        if(result2.isPresent()) {
            EstimatedRobotPose estimatedPose = result2.get();
            poseEstimator.setVisionMeasurementStdDevs(AprilTags.calculateVisionUncertainty(poseX, poseHeading,
                    Constants.VisionConstants.k_cameraToRobot2.getRotation().toRotation2d(),
                    Constants.VisionConstants.k_cameraName2));
            poseEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
        }
    }

    public void renderFieldData() {
        robotWorld.setRobotPose(poseEstimator.getEstimatedPosition());

        SmartDashboard.putData("Rendering/Robot World", robotWorld);
    }
}
