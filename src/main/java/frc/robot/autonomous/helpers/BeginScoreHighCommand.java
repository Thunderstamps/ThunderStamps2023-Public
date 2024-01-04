package frc.robot.autonomous.helpers;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.communications.*;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.arm.commands.*;
import frc.robot.subsystems.arm.commands.gripper.*;

// Does a score cube high command, ends with releasing the gripper
public class BeginScoreHighCommand extends SequentialCommandGroup {
    
    public BeginScoreHighCommand(
            NetworkTableComms nt,
            GripperSubsystem gripperSubsystem,
            JointSubsystem jointSubsystem,
            ExtensionSubsystem extensionSubsystem,
            double delay_sec) {
        this.addCommands(
            new GripGripperCommand(gripperSubsystem),
            new ArmToScoreHighCommand(jointSubsystem, extensionSubsystem),
            new WaitCommand(delay_sec),
            new ReleaseGripperCommand(gripperSubsystem)
        );
    }
}
