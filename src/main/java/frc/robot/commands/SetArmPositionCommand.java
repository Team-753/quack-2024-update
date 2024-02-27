package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class SetArmPositionCommand extends Command {
    private Arm arm;
    private String position;

    public SetArmPositionCommand(Arm kArm, String kPosition) {
        arm = kArm;
        position = kPosition;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setPosition(position);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
