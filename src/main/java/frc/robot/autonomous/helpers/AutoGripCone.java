package frc.robot.autonomous.helpers;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.arm.GripperSubsystem;
import frc.robot.subsystems.arm.commands.gripper.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.intake.commands.*;

public class AutoGripCone extends SequentialCommandGroup {
    
    public AutoGripCone(
        IntakeMode intakeMode,
        GripperSubsystem gripperSubsystem) {

        this.addCommands(
            new SetIntakeMode(intakeMode, IntakeModeEnum.ForwardUp),
            new WaitCommand(0.5),
            new GripGripperCommand(gripperSubsystem),
            new SetIntakeMode(intakeMode, IntakeModeEnum.Off)
        );
    }
}
