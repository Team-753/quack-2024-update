package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.Drive.DriveTrain;

public class DefaultDriveCommand extends Command {
    private CommandJoystick joystick;
    private DriveTrain driveTrain;

    public DefaultDriveCommand(CommandJoystick kJoystick, DriveTrain kDriveTrain) {
        joystick = kJoystick;
        driveTrain = kDriveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        driveTrain.joystickDrive(joystick);
    }
}
