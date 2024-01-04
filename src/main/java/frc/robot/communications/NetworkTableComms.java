package frc.robot.communications;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.communications.gamepieces.*;
import frc.robot.communications.scorelocations.*;
import frc.robot.subsystems.swerve.commands.*;

public class NetworkTableComms {

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private final NetworkTable smartDashboard;
    private final NetworkTable limelight;

    private final NetworkTableEntry gamepieceCube;
    private final NetworkTableEntry gamepieceCone;
    private final NetworkTableEntry scoreLocationHigh;
    private final NetworkTableEntry scoreLocationMid;
    private final NetworkTableEntry extensionHomed;
    private final NetworkTableEntry extensionPosition;
    private final NetworkTableEntry jointHomed;
    private final NetworkTableEntry jointPosition;
    private final NetworkTableEntry vision;
    private final NetworkTableEntry gyro;
    private final NetworkTableEntry tilt;

    private final NetworkTableEntry camMode;
    private final NetworkTableEntry ledMode;
    private final NetworkTableEntry tv;
    private final NetworkTableEntry tx;
    private final NetworkTableEntry pipeline;


    private boolean extensionAtPick = false;

    public NetworkTableComms() {
        this.smartDashboard = inst.getTable("SmartDashboard");
        this.limelight = inst.getTable("limelight");
        
        this.gamepieceCube = smartDashboard.getEntry(" Cube Selected");
        this.gamepieceCone = smartDashboard.getEntry(" Cone Selected");
        this.scoreLocationHigh = smartDashboard.getEntry(" High Selected");
        this.scoreLocationMid = smartDashboard.getEntry(" Mid Selected");
        this.extensionHomed = smartDashboard.getEntry(" Extension Homed");
        this.extensionPosition = smartDashboard.getEntry(" Extension Pos.");
        this.jointHomed = smartDashboard.getEntry(" Joint Homed");
        this.jointPosition = smartDashboard.getEntry(" Joint Pos.");
        this.vision = smartDashboard.getEntry(" Vision");
        this.gyro = smartDashboard.getEntry(" Gyro");
        this.tilt = smartDashboard.getEntry(" Tilt");

        this.camMode = limelight.getEntry("camMode");
        this.ledMode = limelight.getEntry("ledMode");
        this.tv = limelight.getEntry("tv");
        this.tx = limelight.getEntry("tx");
        this.pipeline = limelight.getEntry("pipeline");

        this.setGamepieceCone();
        this.setScoreLocationHigh();

        this.setCamModeVision();
        this.setPipelineTape();
    }

    public void setCamModeDriver() {
        this.camMode.setNumber(1);
        this.ledMode.setNumber(1); // force LEDs off
    }

    public void setCamModeVision() {
        this.camMode.setNumber(0);
        this.ledMode.setNumber(0); // use LED setting from selected pipeline
    }

    public void setPipelineTape() {
        this.pipeline.setNumber(0);
    }

    public void setPipelineTags() {
        this.pipeline.setNumber(1);
    }

    public void setPipelineTapeMid() {
        this.pipeline.setNumber(2);
    }

    public void setPipelineTapeHigh() {
        this.pipeline.setNumber(3);
    }

    public void controlVisionMode(boolean vision) {
        // just for information on the dashboard
        var visionResult = this.getVisionResult();
        if(visionResult.isTargetFound()) {
            this.vision.setNumber(visionResult.getDegrees());
        }
        else {
            this.vision.setNumber(-999);
        }

        if(vision) {
            this.setCamModeVision();
        }
        else {
            this.setCamModeDriver();
        }

        if(this.isGamepieceCone()) {
            if(this.isScoreLocationHigh()) {
                this.setPipelineTapeHigh();
            }
            else {
                this.setPipelineTapeMid();
            }
        }
        else {
            this.setPipelineTags();
        }
    }

    public boolean isTargetFound() {
        return this.getVisionResult().isTargetFound();
    }

    public VisionResult getVisionResult() {
        var validTarget = this.tv.getDouble(0);
        var result = this.tx.getDouble(1000.0);
        if(validTarget >= 0.5 && result >= -90.0 && result <= 90.0) {
            return new VisionResult(true, result);
        }
        return new VisionResult(false, 0);
    }

    public void setGamepieceCube() {
        this.gamepieceCone.setBoolean(false);
        this.gamepieceCube.setBoolean(true);
    }

    public void setGamepieceCone() {
        this.gamepieceCube.setBoolean(false);
        this.gamepieceCone.setBoolean(true);
    }
    
    public boolean isGamepieceCube() {
        return this.gamepieceCube.getBoolean(false);
    }
    
    public boolean isGamepieceCone() {
        return this.gamepieceCone.getBoolean(false);
    }

    public void setScoreLocationHigh() {
        this.scoreLocationMid.setBoolean(false);
        this.scoreLocationHigh.setBoolean(true);
    }

    public void setScoreLocationMid() {
        this.scoreLocationHigh.setBoolean(false);
        this.scoreLocationMid.setBoolean(true);
    }

    public boolean isScoreLocationHigh() {
        return this.scoreLocationHigh.getBoolean(false);
    }

    public boolean isScoreLocationMid() {
        return this.scoreLocationMid.getBoolean(false);
    }

    public boolean getExtensionAtPick() {
        return this.extensionAtPick;
    }

    public void setExtensionAtPick(boolean value) {
        this.extensionAtPick = value;
    }

    public void setExtensionHomed(boolean value) {
        this.extensionHomed.setBoolean(value);
    }

    public void setExtensionPosition(double value) {
        this.extensionPosition.setNumber(value);
    }

    public void setJointHomed(boolean value) {
        this.jointHomed.setBoolean(value);
    }

    public void setJointPosition(double value) {
        this.jointPosition.setNumber(value);
    }

    public void setGyro(double heading, double tiltAngle) {
        this.gyro.setNumber(heading);
        this.tilt.setNumber(tiltAngle);
    }

    public void initializeSwerveCommands(
            GyroEnableCommand gyroEnableCommand,
            GyroDisableCommand gyroDisableCommand) {
        SmartDashboard.putData(gyroEnableCommand);
        SmartDashboard.putData(gyroDisableCommand);
    }

    public void initializeGamepieceCommands(
            CubeGamepieceCommand cubeGamepieceCommand,
            ConeGamepieceCommand coneGamepieceCommand) {
        SmartDashboard.putData(cubeGamepieceCommand);
        SmartDashboard.putData(coneGamepieceCommand);
    }

    public void initializeScoreLocationCommands(
            HighScoreLocationCommand highScoreLocationCommand,
            MidScoreLocationCommand midScoreLocationCommand) {
        SmartDashboard.putData(highScoreLocationCommand);
        SmartDashboard.putData(midScoreLocationCommand);
    }

    public void initializeAutoChooser(SendableChooser<Command> autoPathChooser) {
        SmartDashboard.putData(autoPathChooser);
    }
}
