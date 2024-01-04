package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.intake.*;

// this command is an instant "set" without any delay or further action, useful for autonomous
public class SetIntakeMode extends CommandBase {

    private final IntakeMode intakeMode;
    private IntakeModeEnum intakeModeEnum;

    public SetIntakeMode(IntakeMode intakeMode, IntakeModeEnum intakeModeEnum) {
        this.intakeMode = intakeMode;
        this.intakeModeEnum = intakeModeEnum;
    }

    @Override
    public void initialize() {
        this.intakeMode.setMode(this.intakeModeEnum);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
