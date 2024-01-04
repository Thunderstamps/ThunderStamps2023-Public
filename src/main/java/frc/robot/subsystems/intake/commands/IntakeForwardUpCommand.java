package frc.robot.subsystems.intake.commands;

import java.time.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.communications.NetworkTableComms;
import frc.robot.subsystems.intake.*;

public class IntakeForwardUpCommand extends CommandBase {

    private final IntakeMode intakeMode;
    private final long coneMilliseconds = 3000;
    private final long cubeMilliseconds = 500;
    private Instant startTime;
    private boolean complete;
    private final NetworkTableComms nt;

    public IntakeForwardUpCommand(IntakeMode intakeMode, NetworkTableComms nt) {
        this.intakeMode = intakeMode;
        this.nt = nt;

    }

    @Override
    public void initialize() {
        this.intakeMode.setMode(IntakeModeEnum.ForwardUp);
        this.startTime = Instant.now();
        this.complete = false;
    }

    @Override
    public void execute() {
        var duration = Duration.between(this.startTime, Instant.now());
        if(duration.toMillis() >= this.milliseconds()) {
            this.complete = true;
        }
    }

    private long milliseconds() {
        if(this.nt.isGamepieceCube()) {
            return this.cubeMilliseconds;
        }
        return this.coneMilliseconds;
    }

    @Override
    public boolean isFinished() {
        return this.complete;
    }

    @Override
    public void end(boolean interrupted) {
        if(this.intakeMode.getMode() == IntakeModeEnum.ForwardUp) {
            this.intakeMode.setMode(IntakeModeEnum.Off);
        }
    }
    
}
