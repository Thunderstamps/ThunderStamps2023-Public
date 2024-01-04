package frc.robot.subsystems.swerve.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.swerve.*;


public class SetRobotFieldPositionCommand extends CommandBase {

    private boolean hasRun = false;
    private final SwerveSubsystem swerveSubsystem;
    private final double startingpos_X;
    private final double startingpos_Y;

    public SetRobotFieldPositionCommand(
        SwerveSubsystem swerveSubsystem,
        double startingpos_X,
        double startingpos_Y) {
        this.swerveSubsystem = swerveSubsystem;
        this.startingpos_X = startingpos_X;
        this.startingpos_Y = startingpos_Y;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        this.hasRun = false;
    }

    @Override
    public void execute() {
        var rotation2d = new Rotation2d(0);
        var pose = new Pose2d(this.startingpos_X, this.startingpos_Y, rotation2d);
        this.swerveSubsystem.zeroGyro();
        //this.swerveSubsystem.setGyroTo(this.startingAngle_rad);
        //var gyro_rad = this.swerveSubsystem.getFieldOrientation_rad();
        var gyroRotation = new Rotation2d(0);
        this.swerveSubsystem.SetRobotFieldPosition(pose, gyroRotation);
        System.out.println("Robot field position set");
        this.hasRun = true;
    }

    @Override
    public boolean isFinished() {
        return this.hasRun;
    }
}
