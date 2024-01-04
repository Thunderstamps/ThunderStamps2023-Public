package frc.controllers;

import edu.wpi.first.wpilibj.XboxController;


public interface IXboxController {

    PovService getPovService();

    boolean getXButton();

    boolean getBackButtonPressed();
    boolean getStartButtonPressed();
    boolean getAButtonPressed();
    boolean getBButtonPressed();
    boolean getXButtonPressed();
    boolean getYButtonPressed();

    boolean getLeftBumperPressed();
    boolean getRightBumperPressed();
    double getLeftTriggerAxis();
    double getRightTriggerAxis();
    double getLeftX();
    double getRightX();
    double getLeftY();
    double getRightY( );
    void rumble();

    void printRawAnalogs();

    XboxController getXboxController();
}