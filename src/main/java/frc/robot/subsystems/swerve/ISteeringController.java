/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.swerve;

public interface ISteeringController {

    // The steering controller will automatically "home" when it starts
    // This property reports if the homing sequence is complete.
    boolean isHomed();

    // Call this to move to the target position (units of motor revolutions)
    void setTargetMotorRev(double targetMotorRev);

    // Actual motor distance in full motor revolutions relative to 
    // reference (home) position
    double getMotorRevCount();
    
    // A parameter of the swerve drive, this is the gear ratio
    // of motor turns for full revolution of the steering module
    double getMotorRevsPerSteeringRev();
}
