package frc.robot.subsystems.swerve.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.swerve.*;

public class SwerveScoringModeOnCommand extends CommandBase {
    private SwerveSubsystem swerveSubsystem;

    public SwerveScoringModeOnCommand(
        SwerveSubsystem swerveSubsystem) {
            this.swerveSubsystem = swerveSubsystem;

    }

    @Override
    public void initialize() {
        this.swerveSubsystem.setLowCurrentLimit();
        this.swerveSubsystem.setTargetFieldOrientation_rad(0); // intake away from grid
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
