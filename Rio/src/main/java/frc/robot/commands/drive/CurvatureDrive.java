package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.supers.DifferentialDriveSuper;

public class CurvatureDrive extends CommandBase {
    private final DifferentialDriveSuper difDrive;
    private final CommandXboxController pilotController;

    public CurvatureDrive(DifferentialDriveSuper drive, CommandXboxController driverController) {
        this.difDrive = drive;
        this.pilotController = driverController;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        difDrive.drive.curvatureDrive(-pilotController.getLeftY(), -pilotController.getRightX(), true);
    }
}