package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.arm.commands.*;
import frc.robot.subsystems.arm.commands.gripper.GripGripperCommand;

public class AutoScoreHighCommand extends SequentialCommandGroup {

    public AutoScoreHighCommand(
            GripperSubsystem gripperSubsystem,
            JointSubsystem JointSubsystem,
            ExtensionSubsystem extensionSubsystem) {

        this.addCommands(
            new GripGripperCommand(gripperSubsystem),
            new ArmToScoreHighCommand(JointSubsystem, extensionSubsystem),
            new WaitCommand(0.1),
            new ArmToPickCommand(gripperSubsystem, JointSubsystem, extensionSubsystem)
        );
    }
}
