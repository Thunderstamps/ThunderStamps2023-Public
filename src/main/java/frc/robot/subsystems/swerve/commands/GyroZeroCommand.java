package frc.robot.subsystems.swerve.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.swerve.*;

public class GyroZeroCommand extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;

    public GyroZeroCommand(
        SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.setName("Reset (Zero) Gyro");
    }

    @Override
    public void initialize() {
        this.swerveSubsystem.zeroGyro();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

