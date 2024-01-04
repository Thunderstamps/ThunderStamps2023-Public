package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.intake.*;

public class IntakeForwardDownCommand extends CommandBase {
    private final IntakeMode intakeMode;

    public IntakeForwardDownCommand(IntakeMode intakeMode) {
        this.intakeMode = intakeMode;

    }

    @Override
    public void initialize() {
        this.intakeMode.setMode(IntakeModeEnum.ForwardDown);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
