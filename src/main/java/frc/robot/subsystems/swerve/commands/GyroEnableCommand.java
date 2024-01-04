package frc.robot.subsystems.swerve.commands;


import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.swerve.*;

public class GyroEnableCommand extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;

    public GyroEnableCommand(
        SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.setName("Enable Gyro");
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        this.swerveSubsystem.enableGyro();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
