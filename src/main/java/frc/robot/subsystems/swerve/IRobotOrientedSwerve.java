package frc.robot.subsystems.swerve;

public interface IRobotOrientedSwerve {

    // Call this to execute the swerve drive in velocity mode, 
    // providing a translation vector and a rotation rate.
    void execute(
        IVector2D translationCommand_in_s_rad, 
        double targetRotationRate_rad_s);
}
