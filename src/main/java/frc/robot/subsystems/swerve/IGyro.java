package frc.robot.subsystems.swerve;

public interface IGyro {

    // Returns the heading of the robot relative to the field
    double getFieldOrientation_rad();

    // Returns the rate of change of heading relative to field
    double getRotationRate_rad_s();

    // Set the Field Orientation to zero (useful during practice)
    void resetOffsetToZero_rad();

    /* Modify the Field Orientation by a small adjustment,
    Typically hooked up to Xbox buttons for small heading corrections */
    void adjustOffset_rad(double adjustment_rad);

    /* If the gyro malfunctions we can manually disable it
    which will put us into robot-oriented control only */
    boolean getEnabled();
    void setEnabled(boolean enabled);
    
    double getZAccel_G();

    double getTiltAngle_deg();
    double getRoll_deg();
    double getPitch_deg();

    boolean isTilted();
    boolean isNotTilted();
}
