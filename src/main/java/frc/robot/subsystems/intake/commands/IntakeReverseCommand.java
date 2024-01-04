package frc.robot.subsystems.intake.commands;

import java.time.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.intake.*;

public class IntakeReverseCommand extends CommandBase {
    private final IntakeMode intakeMode;
    private final long milliseconds;
    private IntakeModeEnum originalMode;
    private Instant startTime;
    private boolean complete;

    public IntakeReverseCommand(IntakeMode intakeMode, double seconds) {
        this.intakeMode = intakeMode;
        this.milliseconds = (long)(seconds * 1000);

    }

    @Override
    public void initialize() {
        this.originalMode = this.intakeMode.getMode();
        this.intakeMode.setMode(IntakeModeEnum.ReverseDown);
        this.startTime = Instant.now();
        this.complete = false;
    }

    @Override
    public void execute() {
        var duration = Duration.between(this.startTime, Instant.now());
        if(duration.toMillis() >= this.milliseconds) {
            this.complete = true;
        }
    }

    @Override
    public boolean isFinished() {
        return this.complete;
    }

    @Override
    public void end(boolean interrupted) {
        if(this.intakeMode.getMode() == IntakeModeEnum.ReverseDown) {
            this.intakeMode.setMode(originalMode);
        }
    }
}
