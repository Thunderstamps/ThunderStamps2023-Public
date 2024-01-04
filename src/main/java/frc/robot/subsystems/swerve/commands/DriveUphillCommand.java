package frc.robot.subsystems.swerve.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.swerve.*;

public class DriveUphillCommand extends CommandBase {
    
    private SwerveSubsystem swerveSubsystem;

    public DriveUphillCommand(
        SwerveSubsystem swerveSubsystem) {
            this.swerveSubsystem = swerveSubsystem;

        this.addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        this.swerveSubsystem.executeDriveUphill();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
