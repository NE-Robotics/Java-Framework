package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import frc.robot.simsystems.DifferentialDriveSim;
import static frc.robot.Constants.SystemConstants.k_simulationLogging;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private RobotContainer robotContainer;
  public Robot singleton;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    initializeLogging();
    initializeLocalSevers(true);
    singleton = this;
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();
    robotContainer.setAutonomousCommand();

    // schedule the autonomous command (example)
    if (robotContainer.autoCommand != null) {
      robotContainer.autoCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    if (robotContainer.autoCommand != null) {
      robotContainer.autoCommand.cancel();
    }
    // to change drive just switch the line below from .arcadeDrive to another option
    CommandScheduler.getInstance().schedule(robotContainer.arcadeDrive);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    powerSimulation();
  }

  /**
   * Simulates the bots power system, should be called periodically
   */
  private void powerSimulation() {
    double drawCurrent = DifferentialDriveSim.driveSim.getCurrentDrawAmps();
    double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(drawCurrent);
    SmartDashboard.putNumber("Sim Power System/Bat. Volt.", loadedVoltage);
    SmartDashboard.putNumber("Sim Power System/Current Draw", drawCurrent);
    RoboRioSim.setVInVoltage(loadedVoltage);
  }

  /**
   * Begins AdvantageKit logging and metadata setup
   */
  private void initializeLogging() {
    Logger logger = Logger.getInstance();

    // Record metadata
    logger.recordMetadata("ProjectName", BuildMetadata.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildMetadata.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildMetadata.GIT_SHA);
    logger.recordMetadata("GitDate", BuildMetadata.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildMetadata.GIT_BRANCH);
    logger.recordMetadata("Git Version", Integer.toString(BuildMetadata.GIT_REVISION));
    switch (BuildMetadata.DIRTY) {
      case 0:
        logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        logger.recordMetadata("GitDirty", "Uncommitted changes");
        break;
      default:
        logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    // Running on a real robot, log to a USB stick
    if(Robot.isReal()) {
      logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
      logger.addDataReceiver(new NT4Publisher());
    } else if(Robot.isSimulation() && k_simulationLogging) {
      logger.addDataReceiver(new WPILOGWriter("logs/"));
      logger.addDataReceiver(new NT4Publisher());
    } else if(Robot.isSimulation() && !k_simulationLogging) {
      logger.addDataReceiver(new NT4Publisher());
    }

    // Start AdvantageKit logger
    logger.start();
  }

  /**
   * Any servers used by the code should be initialized here including pathplanner,
   * pathfinding servers, ML detection servers, & any others.
   */
  private void initializeLocalSevers(boolean testingServers) {
    if(testingServers) {
      PathPlannerServer.startServer(5811);
    }
  }
}
