package frc.robot.subsystems.swerve.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.*;

public class FollowTrajectoryCommand extends CommandBase {
    
    private final SwerveSubsystem swerveSubsystem;
    private final Trajectory trajectory;

    // P = proportional gain
    // It controls how aggressively we try to correct the trajectory
    // It's in units of inch/sec per inch.   In other words, how fast do we try to move
    // towards the correct position (in inches per second) for every inch we're away. 
    private final double P = 2.00; 

    private double elapsedTime_sec;
    private double totalTime_sec;
    private double maxError_in;

    public FollowTrajectoryCommand(
            SwerveSubsystem swerveSubsystem,
            Trajectory trajectory) {
        this.swerveSubsystem = swerveSubsystem;
        this.trajectory = trajectory;
        this.addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        this.maxError_in = 0.0;
        this.elapsedTime_sec = 0.0;
        this.totalTime_sec = this.trajectory.getTotalTimeSeconds();
        //System.out.println("Started trajectory, nominal time (s): " + this.totalTime_sec);
    }

    @Override
    public void execute() {
        if(this.elapsedTime_sec <= this.totalTime_sec){
            this.elapsedTime_sec += Robot.SCAN_TIME_S;
        }
        Trajectory.State goal = this.trajectory.sample(this.elapsedTime_sec);

        var nominalTranslation_in_sec = getNominalTranslation_in_sec(goal);
        var idealPosition_in = getIdealPosition_in(goal);
        var actualPosition_in = getActualPosition_in();

        // calculate a vector pointed in the direction we need to go to correct our position towards where we should be
        var error_in = new Vector2D();
        error_in.subtract(idealPosition_in, actualPosition_in);

        this.maxError_in = Math.max(this.maxError_in, error_in.getMagnitude());

        // calculate a correcting velocity 
        var correctionVelocity_in_sec = error_in.getMagnitude() * P;
        var correction_in_sec = Vector2D.FromPolar(
            correctionVelocity_in_sec, 
            error_in.getAngleRadians());

        // find total translation vector (nominal + correction)
        var translation_in_sec = new Vector2D();
        translation_in_sec.add(nominalTranslation_in_sec, correction_in_sec);

        this.swerveSubsystem.executeAutonomousControl(
            translation_in_sec,
            Vector2D.FromXY(0, 0),
            0.0);

        this.counter++;
        if(this.counter >= 4) {
            this.counter = 0;
            //System.out.println("Traj. err: " + Math.round(error_in.getMagnitude()) + ", ang: " + Math.round(Math.toDegrees(error_in.getAngleRadians())));
        }
    }

    private int counter = 0;

    // At this point in the trajectory (goal), this returns the translation vector
    // we *should* be going assuming everything is going perfectly.
    private Vector2D getNominalTranslation_in_sec(Trajectory.State goal) {
        var speed_in_sec = goal.velocityMetersPerSecond; // trajectory is unit-less, and we put everything in with inches
        var angleRadians = goal.poseMeters.getRotation().getRadians();
        return Vector2D.FromPolar(speed_in_sec, angleRadians);
    }

    // This returns where we *should* be on the field if everything is going perfectly.
    private Vector2D getIdealPosition_in(Trajectory.State goal) {
        var goalPosition_in = goal.poseMeters.getTranslation(); // it's not really in meters, all data was input in inches and trajectory is unit-less
        return Vector2D.FromXY(goalPosition_in.getX(), goalPosition_in.getY());
    }

    // This returns where we actually are, based on odometry
    private Vector2D getActualPosition_in() {
        var robotPosition_in = this.swerveSubsystem.getPose().getTranslation();
        return Vector2D.FromXY(robotPosition_in.getX(), robotPosition_in.getY());
    }

    @Override
    public boolean isFinished() {
        return this.elapsedTime_sec >= this.totalTime_sec;
    }

    @Override
    public void end(boolean interrupted) {
        var pose = this.swerveSubsystem.getPose();
        System.out.println("Following Trajectory END, Max error: " + this.maxError_in + ", odo X: " + pose.getX() + ", odo Y: " + pose.getY());
        this.swerveSubsystem.executeAutonomousControl(
            Vector2D.FromXY(0, 0),
            Vector2D.FromXY(0, 0),
            0.0);
    }
}
