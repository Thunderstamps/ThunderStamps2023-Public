package frc.robot.subsystems.arm.commands.gripper;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.arm.GripperSubsystem;
import frc.robot.utilities.OnDelayTimer;

public class ReleaseGripperCommand extends CommandBase {
    
    private final GripperSubsystem gripperSubsystem;
    private final OnDelayTimer ungripTimer = new OnDelayTimer(100);

    public ReleaseGripperCommand(
            GripperSubsystem gripperSubsystem) {
        this.gripperSubsystem = gripperSubsystem;

        this.addRequirements(gripperSubsystem);
    }

    @Override
    public void initialize() {
        this.gripperSubsystem.Release();
        this.ungripTimer.execute(false);
    }

    @Override
    public void execute() {
        this.ungripTimer.execute(true);
    }

    @Override
    public boolean isFinished() {
        return this.ungripTimer.getOutput();
    }
}
