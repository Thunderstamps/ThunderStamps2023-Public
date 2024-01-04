package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ExtensionSubsystem;
import frc.robot.subsystems.arm.JointSubsystem;

public abstract class ArmMotionCommandBase extends SequentialCommandGroup {
    
    protected ArmMotionCommandBase(
            JointSubsystem jointSubsystem,
            ExtensionSubsystem extensionSubsystem) {

        this.addRequirements(jointSubsystem);
        this.addRequirements(extensionSubsystem);
    }
}
