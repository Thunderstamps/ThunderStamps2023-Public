/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve;

public interface IDriveController {

    // Call this to put the drive motor into speed-controlled 
    // mode and attempt to run at the given target speed.
    void executeVelocityMode(double targetMotorRpm);

    // Actual motor speed
    double getMotorRpm();

    // Call this to put the drive motor into position-controlled
    // mode and move to the target position (units of motor revolutions)
    void executePositionMode(double targetMotorRev);

    // Actual motor distance travelled
    double getMotorRevCount();

    // A parameter of the swerve module, this is the gear ratio
    // of motor turns for full revolution of the wheel (~8.0:1.0)
    double getMotorRevsPerWheelRev();

    // A parameter of the swerve module, this is the wheel size
    double getWheelDiameterInches();

    // A parameter of the swerve drive, max motor speed in RPM
    double getMaxSpeedRpm();

    void setLowCurrentLimit();
    void setMaxCurrentLimit();
}
