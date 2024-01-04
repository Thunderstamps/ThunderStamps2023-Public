package frc.robot.autonomous.helpers;

import java.io.*;
import java.nio.file.*;
import java.util.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.trajectory.constraint.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Vector2D;

public abstract class AutoBase extends SequentialCommandGroup {
    
    protected Trajectory LoadTrajectory(String pathname){
        String trajectoryJSON = ("paths/output/" + pathname + ".wpilib.json");
        Trajectory trajectory = new Trajectory();

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
        return trajectory;
    }

    protected Pose2d getFinalPose(Trajectory trajectory) {
        var lastTime = trajectory.getTotalTimeSeconds();
        Trajectory.State goal = trajectory.sample(lastTime);
        return goal.poseMeters;
    }

    protected Vector2D fromPose(Pose2d pose2d) {
        return Vector2D.FromXY(pose2d.getX(), pose2d.getY());
    }
    
    protected Trajectory MakeTrajectory(ArrayList<Vector2D> waypoints, double finalSpeed_in_s) {
        return MakeTrajectory(waypoints, Robot.MAX_ACCELERATION_AUTO_G*386, Robot.MAX_SPEED_IN_SEC, finalSpeed_in_s);
    }
    
    protected Trajectory MakeTrajectory(ArrayList<Vector2D> waypoints, double maxAccel_in_sec2, double maxSpeed_in_sec, double finalSpeed_in_s) {
        var newWaypoints = new ArrayList<Pose2d>();
        for (Vector2D vector2d : waypoints) {
            newWaypoints.add(toPose2D(vector2d, 0, 0, 0));
        }
        return makeTrajectory(newWaypoints, maxAccel_in_sec2, maxSpeed_in_sec, finalSpeed_in_s);
    }
    
    private Trajectory makeTrajectory(ArrayList<Pose2d> waypoints, double maxAccel_in_sec2, double maxSpeed_in_sec, double finalSpeed_in_s) {
        var config = new TrajectoryConfig(maxSpeed_in_sec, maxAccel_in_sec2);
        config.addConstraint(new CentripetalAccelerationConstraint(maxAccel_in_sec2));
        config.setEndVelocity(finalSpeed_in_s);

        var result = TrajectoryGenerator.generateTrajectory(waypoints, config);
        return result;
    }

    protected Pose2d toPose2D(Vector2D vect, double offsetX_in, double offsetY_in, double heading_deg) {
        return new Pose2d(vect.getX() + offsetX_in, vect.getY() + offsetY_in, Rotation2d.fromDegrees(heading_deg));
    }
}
