package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.intake.*;

public class IntakeOffCommand extends CommandBase {
    private final IntakeMode intakeMode;

    public IntakeOffCommand(IntakeMode intakeMode) {
        this.intakeMode = intakeMode;

    }

    @Override
    public void initialize() {
        this.intakeMode.setMode(IntakeModeEnum.Off);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
