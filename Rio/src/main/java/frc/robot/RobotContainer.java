package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.autos.pathPlannerAutoHandler;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.CurvatureDrive;
import frc.robot.commands.drive.TankDrive;
import frc.robot.simsystems.DifferentialDriveSim;
import frc.robot.subsystems.DifferentialDriveReal;
import frc.robot.supers.DifferentialDriveSuper;

import org.jetbrains.annotations.NotNull;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Non-Static Access
  public static RobotContainer singleton;

  // Subsystem Supers
  private DifferentialDriveSuper differentialDrive;

  // Commands
  public ArcadeDrive arcadeDrive;
  public CurvatureDrive curvatureDrive;
  public TankDrive tankDrive;
  public Command autoCommand;

  // Controllers
  private final CommandXboxController pilotController
          = new CommandXboxController(Constants.OIConstants.k_pilotControllerPort);
  private final CommandXboxController operatorController
          = new CommandXboxController(Constants.OIConstants.k_operatorControllerPort);

  // Dashboard inputs
  public static LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    if(Robot.isReal()) {
      assignSubsystems();
    } else  {
      assignSimSystems();
    }
    configureCommands();
    configureAutoChooser(autoChooser);
    new pathPlannerAutoHandler(autoChooser, differentialDrive);
    configureButtonBindings();
    singleton = this;
  }

  /**
   * The function which assigns all real subsystems
   */
  private void assignSubsystems() {
    differentialDrive = new DifferentialDriveReal();
  }

  /**
   * The function which assigns all simulated subsystems aka simsystems
   */
  private void assignSimSystems() {
    differentialDrive = new DifferentialDriveSim();
  }

  /**
   * The function which assigns all commands
   */
  private void configureCommands() {
    arcadeDrive = new ArcadeDrive(differentialDrive, pilotController);
    curvatureDrive = new CurvatureDrive(differentialDrive, pilotController);
    tankDrive = new TankDrive(differentialDrive, pilotController);
  }

  /**
   * A function which assigns auto options to the passed in chooser
   * your manually defined autos should be added here
   * @param chooser an AdvantageKit Logged Dashboard Command Chooser
   */
  private void configureAutoChooser(@NotNull LoggedDashboardChooser<Command> chooser) {
    chooser.addDefaultOption("Do Nothing", new InstantCommand());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to set the autonomous command
   */
  public void setAutonomousCommand() {
    autoCommand = autoChooser.get();
  }
}
