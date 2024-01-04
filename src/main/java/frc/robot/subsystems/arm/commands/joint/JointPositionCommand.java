package frc.robot.subsystems.arm.commands.joint;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.JointSubsystem;

public class JointPositionCommand extends CommandBase {
    
    public JointPositionCommand(
            JointSubsystem jointSubsystem) {

        this.addRequirements(jointSubsystem);
    }
}
