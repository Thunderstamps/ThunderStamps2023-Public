package frc.robot.autonomous.helpers;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.swerve.*;
import frc.robot.utilities.OnDelayTimer;

public class WaitForTip extends CommandBase {
    
    private final IGyro gyro;
    private final double minDegrees;
    private final OnDelayTimer timer;

    public WaitForTip(
            IGyro gyro,
            double minDegrees,
            long delay_ms) {
        this.gyro = gyro;
        this.minDegrees = minDegrees;
        this.timer = new OnDelayTimer(delay_ms);
    }

    @Override
    public void initialize() {
        this.timer.execute(false);
    }

    @Override
    public void execute() {
        this.timer.execute(this.gyro.getTiltAngle_deg() >= this.minDegrees);
    }

    @Override
    public boolean isFinished() {
        return this.timer.getOutput();
    }
}
