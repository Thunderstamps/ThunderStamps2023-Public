package frc.robot.subsystems.swerve;

public interface ISwerveModule {
    
    
    /*This status property echoes the property
    of ISteeringController.isHomed() */
    boolean isHomed();


    /* Call this to execute the module in velocity mode with the given command
    The command is a vector with a magnitude in in/sec and direction in radians
    relative to forward being the front of the robot */
    void executeVelocityMode(IVector2D velocityCommand_in_s_rad);

    /* Returns the actual swerve module velocity in inches per second
    with direction in radians relative to the front of the robot */
    IVector2D getCurrentVelocity_in_s_rad();


    /* Returns the swerve module location relative to the center of mass */
    IVector2D getModulePos_in();

    /* returns the swerve module orientation relative to robot forward */
    double getModuleOrientation_rad();

    /* Returns the Maximum wheel speed allowed in inches per second */
    double getMaxSpeed_in_s();

    void setLowCurrentLimit();
    void setMaxCurrentLimit();
}