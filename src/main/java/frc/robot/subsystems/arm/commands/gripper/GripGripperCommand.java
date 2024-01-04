package frc.robot.subsystems.arm.commands.gripper;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.arm.GripperSubsystem;

public class GripGripperCommand extends CommandBase {
    
    private GripperSubsystem gripperSubsystem;

    public GripGripperCommand(
            GripperSubsystem gripperSubsystem) {
        this.gripperSubsystem = gripperSubsystem;

        this.addRequirements(gripperSubsystem);
    }

    @Override
    public void initialize() {
        this.gripperSubsystem.Grip();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
