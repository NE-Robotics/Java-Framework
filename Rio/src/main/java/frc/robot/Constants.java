package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class SystemConstants {
        // System Constants
        public static final double k_periodicRate = 0.02;
        public static final boolean k_simulationLogging = false;
    }
    public static class OIConstants {
        public static final int k_pilotControllerPort = 0;
        public static final int k_operatorControllerPort = 1;
    }
    public static class DriveConstants {
        // Spark Max Setup
        public static final int k_leftFrontID = 10;
        public static final int k_leftBackID = 11;
        public static final int k_rightFrontID = 12;
        public static final int k_rightBackID = 13;
        public static final int k_currentLimit = 40;
        public static final CANSparkMax.IdleMode k_idleMode = CANSparkMax.IdleMode.kBrake;

        // Kinematics & Estimation
        public static final double k_trackWidth = 0.5900928;
        public static final double k_wheelDiameterMeters = 0.15;

        // Simulation
        public static final DCMotor k_driveMotor = DCMotor.getNEO(2);
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 3;
        public static final double kaVoltSecondsSquaredPerMeter = 0.3;
        public static final double kvVoltSecondsPerRadian = 2;
        public static final double kaVoltSecondsSquaredPerRadian = 0.4;
        public static final LinearSystem<N2, N2, N2> k_drivetrainPlant = LinearSystemId.identifyDrivetrainSystem(
                kvVoltSecondsPerMeter,
                kaVoltSecondsSquaredPerMeter,
                kvVoltSecondsPerRadian,
                kaVoltSecondsSquaredPerRadian);
    }
    public static class EncoderConstants {
        // Encoder Constants
        public static final int k_simEncoderCPR = 1024;
        public static final double k_simEncoderDistancePerPulse = (DriveConstants.k_wheelDiameterMeters * Math.PI) / k_simEncoderCPR;
        public static double realPositionToDistance(double position) {
            return (position / Constants.GearboxConstants.k_driveGearing)
                    * Constants.DriveConstants.k_wheelDiameterMeters * Math.PI;
        }
    }
    public static class GearboxConstants {
        // Gearbox Constants
        public static final double k_driveGearing = 8.25;
    }
    public static class VisionConstants {
        // from center of the bot & ground up
        public final static Transform3d k_cameraToRobot1 =
                new Transform3d(
                        new Translation3d(
                                0.31115,
                                -0.2413,
                                0.4953),
                        new Rotation3d(
                                0,
                                0,
                                0.017));
        public final static Transform3d k_cameraToRobot2 =
                new Transform3d(
                        new Translation3d(
                                -0.31115,
                                -0.2413,
                                0.5207),
                        new Rotation3d(
                                0,
                                0,
                                3.14));
        public static final String k_cameraName1 = "front camera";
        public static final String k_cameraName2 = "back camera";
    }
}
