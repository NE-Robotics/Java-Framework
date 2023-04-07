package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import frc.robot.Constants;
import frc.robot.supers.DifferentialDriveSuper;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import utilities.FileUtilities;

import java.util.HashMap;

import static edu.wpi.first.wpilibj.Filesystem.getDeployDirectory;

public class pathPlannerAutoHandler {
    LoggedDashboardChooser<Command> autoChooser;
    private final DifferentialDriveSuper differentialDrive;
    RamseteAutoBuilder autoBuilder;
    HashMap<String, Command> eventMap = new HashMap<>();

    /**
     * Automatically generates new auto choices from all pathplanner autos deployed to the bot
     */
    public pathPlannerAutoHandler(LoggedDashboardChooser<Command> chooser, DifferentialDriveSuper differentialDrive) {
        this.differentialDrive = differentialDrive;
        this.autoChooser = chooser;
        System.out.println("WARNING: Path Planner Auto Handler is experimental and may not work as expected");
        System.out.println("WARNING: Path Planner Auto Handler currently uses a different coordinate frame than the rest of the robot code.");
        configureAuto();
    }

    private void configureAuto() {
        configureEventMap(eventMap);
        configureRamseteAutoBuilder(eventMap);
        String[] paths = FileUtilities.getFileNamesWithoutExtensions(getDeployDirectory() + "/pathplanner");
        assert paths != null;
        for (String path : paths) {
            addPathToAutoChooser(path);
        }
    }

    private void addPathToAutoChooser(String path) {
        PathPlannerTrajectory autoPath = PathPlanner.loadPath(path, new PathConstraints(4, 2));
        FollowPathWithEvents autoWithEvents = new FollowPathWithEvents(
                autoBuilder.followPath(autoPath),
                autoPath.getMarkers(),
                eventMap
        );
        autoChooser.addOption(path, autoWithEvents);
    }

    private void configureEventMap(HashMap<String, Command> eventMap) {
        System.out.println("WARNING: Path Planner Auto Handler Event Map is experimental and may not work as expected");
        eventMap.put("Waypoint 1", new PrintCommand("Passed marker 1"));
        eventMap.put("Waypoint 2", new PrintCommand("Passed marker 2"));
    }
    private void configureRamseteAutoBuilder(HashMap<String, Command> eventMap) {
        autoBuilder = new RamseteAutoBuilder(
                differentialDrive.poseEstimator::getEstimatedPosition,
                differentialDrive::setPose,
                new RamseteController(1.5, 5),
                differentialDrive.driveKinematics,
                new SimpleMotorFeedforward(
                        Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter
                ),
                differentialDrive::getWheelSpeeds,
                new PIDConstants(1, 1, 0.5),
                differentialDrive::voltageDrive,
                eventMap,
                true,
                differentialDrive
        );
    }
}
