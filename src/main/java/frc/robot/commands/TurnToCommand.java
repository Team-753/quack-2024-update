package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Config;
import frc.robot.subsystems.Drive.DriveTrain;

public class TurnToCommand extends Command {
    private TrapezoidProfile.State targetState; // in radians
    private ProfiledPIDController angleController;
    private DriveTrain driveTrain;

    public TurnToCommand(DriveTrain kDriveTrain, Rotation2d targetAngle) {
        driveTrain = kDriveTrain;
        angleController = new ProfiledPIDController(Config.AutonomousConstants.turnTokP, Config.AutonomousConstants.turnTokI, Config.AutonomousConstants.turnTokD, Config.DriveConstants.turnControllerConstraints);
        angleController.setTolerance(Config.DriveConstants.turnCommandAngleTolerance, Config.DriveConstants.turnCommandVelocityTolerance); // +/- 0.5 degrees (yes value is converted to radians)
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        targetState = new TrapezoidProfile.State(targetAngle.getRadians(), 0);
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        driveTrain.setChassisSpeeds(0.0, 0.0, angleController.calculate(driveTrain.getEstimatedPose().getRotation().getRadians(), targetState), true);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stationary();
    }

    @Override
    public boolean isFinished() {
        return angleController.atSetpoint();
    }
}
