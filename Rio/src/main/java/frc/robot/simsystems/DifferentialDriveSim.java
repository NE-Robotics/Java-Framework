package frc.robot.simsystems;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.supers.DifferentialDriveSuper;
import utilities.vision.AprilTags;

public class DifferentialDriveSim extends DifferentialDriveSuper {
    // The left-side drive encoder
    private final Encoder leftEncoder = new Encoder(0, 1, false);

    // The right-side drive encoder
    private final Encoder rightEncoder = new Encoder(2, 3, false);

    // Simulation of drivetrain & encoders
    private EncoderSim leftEncoderSim;
    private EncoderSim rightEncoderSim;
    public static DifferentialDrivetrainSim driveSim;
    private final Field2d trueWorld = new Field2d();

    public DifferentialDriveSim() {
        configureMotors();
        setupEncoders();
        resetEncoders();
        setupDrive();
    }

    @Override
    public void simulationPeriodic() {
        handleDriveSimulation();
        handleSensorSimulation();
        renderFieldData();
        AprilTags.calculateVisionUncertainty(poseEstimator.getEstimatedPosition().getX(),
                poseEstimator.getEstimatedPosition().getRotation(),
                Constants.VisionConstants.k_cameraToRobot1.getRotation().toRotation2d(),
                Constants.VisionConstants.k_cameraName1);
        AprilTags.calculateVisionUncertainty(poseEstimator.getEstimatedPosition().getX(),
                poseEstimator.getEstimatedPosition().getRotation(),
                Constants.VisionConstants.k_cameraToRobot2.getRotation().toRotation2d(),
                Constants.VisionConstants.k_cameraName2);
    }

    private void setupEncoders() {
        leftEncoder.setDistancePerPulse(Constants.EncoderConstants.k_simEncoderDistancePerPulse);
        rightEncoder.setDistancePerPulse(Constants.EncoderConstants.k_simEncoderDistancePerPulse);
        leftEncoderSim = new EncoderSim(leftEncoder);
        rightEncoderSim = new EncoderSim(rightEncoder);
    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    private void setupDrive() {
        driveSim = new DifferentialDrivetrainSim(
                Constants.DriveConstants.k_drivetrainPlant,
                Constants.DriveConstants.k_driveMotor,
                Constants.GearboxConstants.k_driveGearing,
                Constants.DriveConstants.k_trackWidth,
                Constants.DriveConstants.k_wheelDiameterMeters/2,
                VecBuilder.fill(0.005, 0.005, 0.0001, 0.05, 0.05, 0.005, 0.005)
        );

        driveSim.setPose(driveOdometry.getPoseMeters());

         poseEstimator = new DifferentialDrivePoseEstimator(
                driveKinematics,
                driveOdometry.getPoseMeters().getRotation(),
                getLeftEncoderDistanceMeters(),
                getRightEncoderDistanceMeters(),
                driveOdometry.getPoseMeters()
        );
    }

    @Override
    public double getLeftEncoderDistanceMeters() {
        return leftEncoder.getDistance();
    }

    @Override
    public double getRightEncoderDistanceMeters() {
        return rightEncoder.getDistance();
    }

    @Override
    public void voltageDrive(double leftVolts, double rightVolts) {
        // Equivalent of set voltage for simulation
        leftMotors.set(leftVolts / RobotController.getBatteryVoltage());
        rightMotors.set(rightVolts / RobotController.getBatteryVoltage());
        drive.feed();
    }

    @Override
    public void setPose(Pose2d pose) {
        driveSim.setPose(pose);
        driveOdometry.resetPosition(getHeading(), 0, 0, pose);
        poseEstimator.resetPosition(getHeading(), 0, 0, pose);
    }

    private void handleDriveSimulation() {
        driveSim.setInputs(
                leftMotors.get() * RobotController.getBatteryVoltage(),
                rightMotors.get() * RobotController.getBatteryVoltage()
        );
        driveSim.update(Constants.SystemConstants.k_periodicRate);

        poseEstimator.update(
                getHeading(),
                getLeftEncoderDistanceMeters(),
                getRightEncoderDistanceMeters()
        );
    }

    private void handleSensorSimulation() {
        leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
        leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
        rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
        rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());

        int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
        angle.set(-driveSim.getHeading().getDegrees());
    }

    public void renderFieldData() {
        robotWorld.setRobotPose(poseEstimator.getEstimatedPosition());
        trueWorld.setRobotPose(driveSim.getPose());

        SmartDashboard.putData("Rendering/Robot World", robotWorld);
        SmartDashboard.putData("Rendering/True World", trueWorld);
    }
}
