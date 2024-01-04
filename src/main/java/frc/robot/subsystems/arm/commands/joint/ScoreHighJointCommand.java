package frc.robot.subsystems.arm.commands.joint;

import frc.robot.subsystems.arm.JointSubsystem;

public class ScoreHighJointCommand extends JointPositionCommand {

    private JointSubsystem jointSubsystem;
    private double targetPosition_rot;

    public ScoreHighJointCommand(
            JointSubsystem jointSubsystem) {
        super(jointSubsystem);
        this.jointSubsystem = jointSubsystem;
    }

    @Override
    public void initialize() {
        this.targetPosition_rot = this.jointSubsystem.MoveToScoreHigh();
    }

    @Override
    public boolean isFinished() {
        return this.jointSubsystem.isAtPosition(this.targetPosition_rot, JointSubsystem.POSITION_TOLERANCE_UP_ROT);
    }
    
}
