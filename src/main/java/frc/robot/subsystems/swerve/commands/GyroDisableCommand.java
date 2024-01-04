package frc.robot.subsystems.swerve.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.swerve.*;

public class GyroDisableCommand extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;

    public GyroDisableCommand(
        SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.setName("Disable Gyro");
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        this.swerveSubsystem.disableGyro();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
