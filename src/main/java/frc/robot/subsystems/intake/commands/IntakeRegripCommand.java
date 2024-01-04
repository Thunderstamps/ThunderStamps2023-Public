package frc.robot.subsystems.intake.commands;

import java.time.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.intake.*;

public class IntakeRegripCommand extends CommandBase {

    private final IntakeMode intakeMode;
    private final long delayMilliseconds = 200;
    private Instant startTime;
    private boolean complete;

    public IntakeRegripCommand(IntakeMode intakeMode) {
        this.intakeMode = intakeMode;
    }

    @Override
    public void initialize() {
        this.intakeMode.setMode(IntakeModeEnum.Regrip);
        this.startTime = Instant.now();
        this.complete = false;
    }

    @Override
    public void execute() {
        var duration = Duration.between(this.startTime, Instant.now());
        if(duration.toMillis() >= this.delayMilliseconds) {
            this.complete = true;
        }
    }

    @Override
    public boolean isFinished() {
        return this.complete;
    }

    @Override
    public void end(boolean interrupted) {
        if(this.intakeMode.getMode() == IntakeModeEnum.Regrip) {
            this.intakeMode.setMode(IntakeModeEnum.ForwardUp);
        }
    }
    
}
