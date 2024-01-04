package frc.robot.subsystems.arm.commands.joint;

import frc.robot.subsystems.arm.JointSubsystem;

public class PickJointCommand extends JointPositionCommand {

    private JointSubsystem jointSubsystem;

    public PickJointCommand(
            JointSubsystem jointSubsystem) {
        super(jointSubsystem);
        this.jointSubsystem = jointSubsystem;
        
    }

    @Override
    public void initialize() {
        this.jointSubsystem.MoveToPick();
    }

    @Override
    public boolean isFinished() {
        return this.jointSubsystem.isAtPick();
    }
    
}
