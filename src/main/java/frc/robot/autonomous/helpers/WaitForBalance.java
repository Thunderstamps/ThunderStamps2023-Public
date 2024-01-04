package frc.robot.autonomous.helpers;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.IGyro;

public class WaitForBalance extends CommandBase {
    
    private IGyro gyro;
    private double maxDegrees;

    public WaitForBalance(
            IGyro gyro,
            double maxDegrees) {
        this.gyro = gyro;
        this.maxDegrees = maxDegrees;

    }

    @Override
    public boolean isFinished() {
        return this.gyro.getTiltAngle_deg() <= maxDegrees;
    }
}
