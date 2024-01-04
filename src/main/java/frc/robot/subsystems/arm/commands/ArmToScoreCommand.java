package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.communications.*;
import frc.robot.subsystems.arm.*;

public class ArmToScoreCommand extends ConditionalCommand {
    
    public ArmToScoreCommand(
            NetworkTableComms nt,
            JointSubsystem jointSubsystem, 
            ExtensionSubsystem extensionSubsystem) {

        super(new ArmToScoreMidCommand(jointSubsystem, extensionSubsystem), 
            new ArmToScoreHighCommand(jointSubsystem, extensionSubsystem), 
            nt::isScoreLocationMid);

    }

    
}
