// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.controllers.*;
import frc.robot.communications.*;
import frc.robot.communications.gamepieces.*;
import frc.robot.subsystems.swerve.*;

public class Robot extends TimedRobot {
    private NetworkTableComms nt = new NetworkTableComms();
    private final LedController ledController = new LedController();
    private IXboxController xboxController = new XboxControllerSCUF(0);
    private Joystick joystick = new Joystick(1);
    private IGyro gyro;
    private SwerveSubsystem swerveSubsystem;
    //private PhotonCamera camera = new PhotonCamera("OV9281");
    public static final double MAX_ACCELERATION_AUTO_G = 0.250;
    public static final double MAX_ACCELERATION_TELEOP_G = 0.8;
    public static final double SCAN_TIME_S = 0.020;
    public static final double ROTATION_P = 5.0;
    public static final double MAX_ROTATION_RATE_RAD_S = Math.toRadians(220); // was 300 in 2022
    public static final double MAX_SPEED_IN_SEC = 220.0; // L1: 125.0, COMP: 167 in 2022, L4 is 200 in 2023
    
    public static final boolean ENABLE_SWERVE = true;
    public static final boolean ENABLE_COMPRESSOR = true;

    private final PneumaticHub pneumaticHub = new PneumaticHub(2);
    private RobotContainer m_robotContainer;
    private Command m_autonomousCommand;

    public Robot() {
        super(SCAN_TIME_S);
    }

    @Override
    public void robotInit() {

        CameraServer.startAutomaticCapture("Operator Camera", 0);

        // Primary NavX2 has a ScaleFactor of 1.0315
        // Backup Navx2 has a ScaleFactor of 0.9050
        // If installing the NavX classic board, use the NavXMxpGyro class instead of the NavX2MxpGyro class
        gyro = new NavX2MxpGyro(1.0315); 
        gyro.setEnabled(true);

        this.swerveSubsystem = new SwerveSubsystem(xboxController, joystick, gyro, nt);

        m_robotContainer = new RobotContainer(
            this.nt,
            this.xboxController,
            this.joystick,
            this.swerveSubsystem,
            this.gyro
        );
    }

    @Override
    public void robotPeriodic() {
        
        this.nt.controlVisionMode(this.xboxController.getXboxController().getBButton()
            || this.isAutonomous());
        
        // allows swerve to home, and runs odometry, etc.
        if(ENABLE_SWERVE) {
            this.swerveSubsystem.RunRobotPeriodic();
            //this.swerveSubsystem.printSteeringOrientations();
        }

        if(ENABLE_COMPRESSOR) {
            pneumaticHub.enableCompressorDigital();
        }
        else {
            pneumaticHub.disableCompressor();
        }

        this.m_robotContainer.robotPeriodic();

        this.gyro.getTiltAngle_deg();

        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        this.m_robotContainer.disabledInit();

        this.xboxController.getXboxController().setRumble(RumbleType.kLeftRumble, 0);
        this.xboxController.getXboxController().setRumble(RumbleType.kRightRumble, 0);
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        this.m_robotContainer.autonomousInit();

        if(ENABLE_SWERVE) {
            // sets accel limits, etc.
            this.swerveSubsystem.autonomousInit();
            
            m_autonomousCommand = m_robotContainer.getAutonomousCommand();

            // schedule the autonomous command (example)
            if (m_autonomousCommand != null) {
              m_autonomousCommand.schedule();
            }
        }
    }

    @Override
    public void autonomousPeriodic() {
        //System.out.println(swerveSubsystem.getPose());

        this.ledController.SetOff();
    }

    @Override
    public void teleopInit() {
        this.m_robotContainer.teleopInit();

        if(ENABLE_SWERVE) {
            // sets accel limits, etc.
            this.swerveSubsystem.teleopInit();
        }

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
  
    }

    @Override
    public void teleopPeriodic() {
        if(this.nt.isGamepieceCone()) {
            this.ledController.SetCone();
        }
        else {
            this.ledController.SetCube();
        }

        if(ENABLE_SWERVE) {
            this.swerveSubsystem.executeOperatorControl();
            //this.swerveSubsystem.runSwerveRobotOrientedForwardSlow();
        }

        //System.out.println(swerveSubsystem.getPose());
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
}
