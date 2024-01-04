package frc.robot.subsystems.swerve;

import java.util.ArrayList;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.controllers.IXboxController;
import frc.robot.Robot;
import frc.robot.communications.NetworkTableComms;

public class SwerveSubsystem extends SubsystemBase{

    private static final double TWIST_DEG = -0.0;
    private static final double TWIST_RAD = Math.toRadians(TWIST_DEG);

    private final IXboxController xbox;
    private final SteeringController steeringController1, steeringController2, steeringController3, steeringController4, steeringController5;
    private final DriveController driveController1, driveController2, driveController3, driveController4, driveController5;
    private final SwerveModule swerveModule1, swerveModule2, swerveModule3, swerveModule4, swerveModule5;
    private final RobotOrientedSwerve robotOrientedSwerve;
    private final FieldOrientedSwerve fieldOrientedSwerve;
    ArrayList<ISwerveModule> swerveModules = new ArrayList<ISwerveModule>();
    private final IGyro gyro;
    private double targetFieldOrientation_rad; 
    private final SwerveDriveKinematics swerveDriveKinematics;
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private Pose2d pose = new Pose2d();
    private boolean homed = false;
    private boolean fineHeading = false;
    private Joystick joystick;
    private NetworkTableComms nt;
    
    public SwerveSubsystem(
        IXboxController xbox,
        Joystick joystick,
        IGyro gyro,
        NetworkTableComms nt) {
        this.xbox = xbox;
        this.joystick = joystick;
        this.gyro = gyro;
        this.nt = nt;
        steeringController1 = new SteeringController(21, 11, 973);
        steeringController2 = new SteeringController(22, 12, 1791);
        steeringController3 = new SteeringController(23, 13, 1425);
        steeringController4 = new SteeringController(24, 14, 2885);
        steeringController5 = new SteeringController(25, 15, 3658);

        driveController1 = new DriveController(31, 5.14, 1.0);
        driveController2 = new DriveController(32, 5.14, 9.0);
        driveController3 = new DriveController(33, 5.14, 5.0);
        driveController4 = new DriveController(34, 5.14, 9.0);
        driveController5 = new DriveController(35, 5.14, 1.0);
        
        var position1 = Vector2D.FromXY(9.402, 14.875);
        var position2 = Vector2D.FromXY(-4.098, 14.875);
        var position3 = Vector2D.FromXY(-17.598, 0);
        var position4 = Vector2D.FromXY(-4.098, -14.875);
        var position5 = Vector2D.FromXY(9.402, -14.875);

        position1.rotate(TWIST_RAD);
        position2.rotate(TWIST_RAD);
        position3.rotate(TWIST_RAD);
        position4.rotate(TWIST_RAD);
        position5.rotate(TWIST_RAD);

        swerveModule1 = new SwerveModule(steeringController1, driveController1, position1, TWIST_RAD);
        swerveModule2 = new SwerveModule(steeringController2, driveController2, position2, TWIST_RAD);
        swerveModule3 = new SwerveModule(steeringController3, driveController3, position3, TWIST_RAD);
        swerveModule4 = new SwerveModule(steeringController4, driveController4, position4, TWIST_RAD);
        swerveModule5 = new SwerveModule(steeringController5, driveController5, position5, TWIST_RAD);

        swerveModules.add(swerveModule1);
        swerveModules.add(swerveModule2);
        swerveModules.add(swerveModule3);
        swerveModules.add(swerveModule4);
        swerveModules.add(swerveModule5);

        robotOrientedSwerve = new RobotOrientedSwerve(swerveModules);
        
        fieldOrientedSwerve = new FieldOrientedSwerve(
            robotOrientedSwerve, 
            gyro, 
            Robot.MAX_SPEED_IN_SEC,
            Robot.MAX_ACCELERATION_TELEOP_G * 386, 
            Robot.MAX_ROTATION_RATE_RAD_S, 
            Robot.SCAN_TIME_S, 
            Robot.ROTATION_P);
        
        swerveDriveKinematics = new SwerveDriveKinematics(
            swerveModule1.getTranslation2d(),
            swerveModule2.getTranslation2d(),
            swerveModule3.getTranslation2d(),
            swerveModule4.getTranslation2d(),
            swerveModule5.getTranslation2d()
        );
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
            swerveDriveKinematics,
            new Rotation2d(), // gyro angle
            this.gSwerveModulePositions(), 
            new Pose2d(0.0, 0.0, new Rotation2d()),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // State measurement standard deviations. X, Y, theta.
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01) // Global measurement standard deviations. X, Y, and theta.
            );
    }

    public void RunRobotPeriodic() {

        if(!homed) {
            steeringController1.tryHoming();
            steeringController2.tryHoming();
            steeringController3.tryHoming();
            steeringController4.tryHoming();
            steeringController5.tryHoming();

            if(steeringController1.isHomed() && steeringController2.isHomed() && steeringController3.isHomed() && steeringController4.isHomed() && steeringController5.isHomed()) {
                this.homed = true;
            }
        }

        if(this.homed) {
            this.runSwerveOdometry();
        }
    }

    public void executeAutoLineup() {
        if(this.gyro.getEnabled()) {
            this.execute(
                Vector2D.FromXY(0, 0),
                Vector2D.FromXY(0, 0), 
                0, 
                true);
        } 
        else {
            this.execute(Vector2D.FromXY(0, 0), Vector2D.FromXY(0, 0), 0.0, false);
        }
    }

    public void executeDriveUphill() {
        var pitch_deg = this.gyro.getPitch_deg();
        if(this.gyro.getEnabled() && Math.abs(pitch_deg) > 9.0) {
            var contSpeed_in_s = 10.0;
            var speed_in_s = pitch_deg > 0 ? contSpeed_in_s : -contSpeed_in_s;
            this.execute(
                Vector2D.FromXY(0, 0),
                Vector2D.FromXY(0, speed_in_s), 
                0, 
                false);
        } 
        else {
            this.execute(Vector2D.FromXY(0, 0), Vector2D.FromXY(0, 0), 0.0, false);
        }
    }

    private void execute(
            Vector2D fieldOrientedTranslationCommand_in_s_rad,
            Vector2D robotOrientedTranslationCommand_in_s_rad,
            double robotOrientedRotationCommand_s_rad,
            boolean autolineup) {

        var anyManualCommand = false;

        if (robotOrientedRotationCommand_s_rad !=0.0) {
            var diff_rad = robotOrientedRotationCommand_s_rad * Robot.SCAN_TIME_S;
            this.targetFieldOrientation_rad += diff_rad;
            anyManualCommand = true;
        }

        if (xbox.getLeftBumperPressed()) {
            var prevSnap_deg = nearest45_deg(this.targetFieldOrientation_rad);
            var newHeading_rad = Math.toRadians(prevSnap_deg + 45.0);
            this.targetFieldOrientation_rad = SwerveUtil.limitAngleFromNegativePItoPI_rad(newHeading_rad);
            anyManualCommand = true;
        }

        if (xbox.getRightBumperPressed()) {
            var prevSnap_deg = nearest45_deg(this.targetFieldOrientation_rad);
            var newHeading_rad = Math.toRadians(prevSnap_deg - 45.0);
            this.targetFieldOrientation_rad = SwerveUtil.limitAngleFromNegativePItoPI_rad(newHeading_rad);
            anyManualCommand = true;
        }

        var headingVector_rad = this.getxboxControllerRightStick();
        var headingVectorMagnitide = headingVector_rad.getMagnitude();
        if(headingVectorMagnitide > 0.5) {
            if(this.xbox.getXboxController().getRightStickButton()) {
                this.fineHeading = true; // if we click, remember we're in fine heading control 
            }

            if(this.fineHeading) {
                this.targetFieldOrientation_rad = headingVector_rad.getAngleRadians();
            }
            else {
                var snapHeading_deg = nearest90_deg(headingVector_rad.getAngleRadians());
                this.targetFieldOrientation_rad = Math.toRadians(snapHeading_deg);
            }
            anyManualCommand = true;
        }
        else {
            this.fineHeading = false;
        }

        if(!anyManualCommand) {
            // auto align
            if(this.xbox.getXboxController().getBButton() || autolineup) {
                var visionResult = this.nt.getVisionResult();
                if(visionResult.isTargetFound()) {
                    var currentHeading_rad = this.gyro.getFieldOrientation_rad();
                    var currentHeading_deg = Math.toDegrees(currentHeading_rad);

                    var visionTargetFieldOriented_deg = visionResult.getDegrees() - currentHeading_deg;

                    var cubeGain = 4.5;
                    var coneGain = 6.5;
                    var gain = this.nt.isGamepieceCone() ? coneGain : cubeGain; // P gain
                    var y = visionTargetFieldOriented_deg * gain; 
                    var max = 30;
                    var min = 4;
                    if(y > max) {
                        y= max;
                    }
                    else if(y < -max) {
                        y = -max;
                    }

                    var driveInSlowThreshold_deg = 8.0;
                    var driveInSlow = visionTargetFieldOriented_deg < driveInSlowThreshold_deg && visionTargetFieldOriented_deg > -driveInSlowThreshold_deg;
                    var slowSpeed_in_sec = 15.0;

                    var driveInFastThreshold_deg = 2.0;
                    var driveInFast = visionTargetFieldOriented_deg < driveInFastThreshold_deg && visionTargetFieldOriented_deg > -driveInFastThreshold_deg;
                    var fastSpeed_in_sec = 30.0;


                    if(!driveInFast) {
                        if(y < min  && y > min/2.0) {
                            y = min;
                        }
                        else if(y > -min  && y < -min/2.0) {
                            y = -min;
                        }

                        var x = driveInSlow ? -slowSpeed_in_sec : 0;
                        fieldOrientedTranslationCommand_in_s_rad = Vector2D.FromXY(x, y);
                        this.targetFieldOrientation_rad = 0;
                    } 
                    else {
                        fieldOrientedTranslationCommand_in_s_rad = Vector2D.FromXY(-fastSpeed_in_sec, y/4.0);
                        this.targetFieldOrientation_rad = -Math.toRadians(visionTargetFieldOriented_deg);
                    }
                }
            }
        }

        var modifiedTargetOrientation_rad = targetFieldOrientation_rad;
        if(modifiedTargetOrientation_rad > Math.toRadians(20.0) || modifiedTargetOrientation_rad < Math.toRadians(-20.0)) {
            modifiedTargetOrientation_rad -= TWIST_RAD;
        }

        this.fieldOrientedSwerve.execute(
            fieldOrientedTranslationCommand_in_s_rad, 
            modifiedTargetOrientation_rad,
            robotOrientedTranslationCommand_in_s_rad, 
            robotOrientedRotationCommand_s_rad);
    }

    private double nearest90_deg(double angle_rad) {
        var limitedAngle_rad = SwerveUtil.limitAngleFromNegativePItoPI_rad(angle_rad);
        var angle_deg = Math.toDegrees(limitedAngle_rad);
        if(angle_deg > 180.0-45.0 || angle_deg < -180.0+45.0) {
            return 180.0;
        }
        if(angle_deg <= 45.0 && angle_deg >= -45.0) {
            return 0.0;
        }
        if(angle_deg > 0) {
            return 90.0;
        }
        return -90.0;
    }

    public double nearest45_deg(double angle_rad) {
        var limitedAngle_rad = SwerveUtil.limitAngleFromNegativePItoPI_rad(angle_rad);
        var angle_deg = Math.toDegrees(limitedAngle_rad);
        if(angle_deg >= -22.5 && angle_deg <= 22.5) {
            return 0.0;
        }
        if(angle_deg >= -45-22.5 && angle_deg <= -45+22.5) {
            return -45.0;
        }
        if(angle_deg >= -90-22.5 && angle_deg <= -90+22.5) {
            return -90.0;
        }
        if(angle_deg >= -135-22.5 && angle_deg <= -135+22.5) {
            return -135.0;
        }
        if(angle_deg >= 45-22.5 && angle_deg <= 45+22.5) {
            return 45.0;
        }
        if(angle_deg >= 90-22.5 && angle_deg <= 90+22.5) {
            return 90.0;
        }
        if(angle_deg >= 135-22.5 && angle_deg <= 135+22.5) {
            return 135.0;
        }
        return 180.0;
    }
    
    private Vector2D getRobotOrientedTranslationCommand_in_s() {
        var robotOrientedTranslation = Vector2D.FromXY(0, 0);
        if(!this.gyro.getEnabled()) {
            robotOrientedTranslation = getxboxControllerLeftStick();
        }
        if(this.joystickOverride()) {
            robotOrientedTranslation = getJoystick();
        }
        robotOrientedTranslation.squareMagnitude();
        double newMagnitude = robotOrientedTranslation.getMagnitude() * Robot.MAX_SPEED_IN_SEC;
        robotOrientedTranslation.setMagnitude(newMagnitude);
        if (robotOrientedTranslation.getMagnitude() < 0.5) {
            robotOrientedTranslation.setMagnitude(0.0);
        }
        return robotOrientedTranslation;
    }


    private Vector2D getFieldOrientedTranslationCommand_in_s() {
        Vector2D input;
        double maxSpeed_inSec;
        input = getxboxControllerLeftStick();
        maxSpeed_inSec = Robot.MAX_SPEED_IN_SEC;

        input.squareMagnitude();
        var newMagnitude = input.getMagnitude() * maxSpeed_inSec;
        Vector2D fieldOrientedTranslation = Vector2D.FromPolar(newMagnitude, input.getAngleRadians());
        if(fieldOrientedTranslation.getMagnitude() < 0.2) {
            fieldOrientedTranslation.setMagnitude(0.0);
        }

        return fieldOrientedTranslation;
    }
    private double getRobotOrientedRotationCommand_rad_s() {
        var leftTrigger = xbox.getLeftTriggerAxis();
        var rightTrigger = xbox.getRightTriggerAxis();
        var triggerMagnitude = (leftTrigger - rightTrigger);
        var triggerMagnitudeSquared = triggerMagnitude * Math.abs(triggerMagnitude);

        if(this.joystickOverride()) {
            var joystickRotate = -joystick.getZ(); // joystick twist rotation
            var joystickRateSquared = joystickRotate * Math.abs(joystickRotate);
            triggerMagnitudeSquared += joystickRateSquared;
        }

        return triggerMagnitudeSquared * Robot.MAX_ROTATION_RATE_RAD_S * 0.20;
    }

    private boolean joystickOverride() {
        return joystick.getRawButton(9) || joystick.getRawButton(10);
    }
    
    private Vector2D getxboxControllerRightStick() {
        var x = xbox.getRightX();
        if(Math.abs(x) < 0.05) {
            x = 0.0;
        }
        var y = xbox.getRightY();
        if(Math.abs(y) < 0.05) {
            y = 0.0;
        }
        return Vector2D.FromXY(-y, -x);
    }

    private Vector2D getxboxControllerLeftStick() {
        var x = xbox.getLeftX();
        if(Math.abs(x) < 0.05) {
            x = 0.0;
        }
        var y = xbox.getLeftY();
        if(Math.abs(y) < 0.05) {
            y = 0.0;
        }
        return Vector2D.FromXY(-y, -x);
    }

    private Vector2D getJoystick() {
        var x = joystick.getX();
        if(Math.abs(x) < 0.05) {
            x = 0.0;
        }
        var y = joystick.getY();
        if(Math.abs(y) < 0.05) {
            y = 0.0;
        }
        return Vector2D.FromXY(-y, -x);
    }

    public void executeOperatorControl() {
        if(this.gyro.getEnabled()) {
            this.runSwerveFieldOriented();
        }
        else {
            this.runSwerveRobotOriented();
        }
    }

    // private void xStop() {
    //     this.xStop(this.swerveModule1, 45);
    //     this.xStop(this.swerveModule2, 135);
    //     this.xStop(this.swerveModule3, 0);
    //     this.xStop(this.swerveModule4, -135);
    //     this.xStop(this.swerveModule5, -45);
    // }

    // private void xStop(ISwerveModule swerveModule, double direction_deg) {
    //     var smallVector = Vector2D.FromPolar(0.1, Math.toRadians(direction_deg));
    //     swerveModule.executeVelocityMode(smallVector);
    // }

    private void runSwerveFieldOriented() {
        var fieldOrientedTranslationCommand_in_s_rad = getFieldOrientedTranslationCommand_in_s();
        var robotOrientedTranslationCommand_in_s_rad = getRobotOrientedTranslationCommand_in_s();
        var robotOrientedRotationCommand_in_s_rad = getRobotOrientedRotationCommand_rad_s();
        this.execute(
            fieldOrientedTranslationCommand_in_s_rad,
            robotOrientedTranslationCommand_in_s_rad,
            robotOrientedRotationCommand_in_s_rad,
            false
        );
    }
    
    private void runSwerveRobotOriented() {
        var robotOrientedTranslationCommand_in_s = getRobotOrientedTranslationCommand_in_s();
        var robotOrientedRotationCommand_rad_s = getRobotOrientedRotationCommand_rad_s();
    
        robotOrientedSwerve.execute(
            robotOrientedTranslationCommand_in_s, 
            robotOrientedRotationCommand_rad_s);
    }

    // this is for calibration
    public void runSwerveRobotOrientedForwardSlow() {
        this.gyro.setEnabled(false);
        robotOrientedSwerve.execute(
            Vector2D.FromPolar(10.0, 0.0), 
            0.0);
    }

    public void printSteeringOrientations() {
        var m1 = this.steeringController1.getAbsoluteCANCoderPosition();
        var m2 = this.steeringController2.getAbsoluteCANCoderPosition();
        var m3 = this.steeringController3.getAbsoluteCANCoderPosition();
        var m4 = this.steeringController4.getAbsoluteCANCoderPosition();
        var m5 = this.steeringController5.getAbsoluteCANCoderPosition();

        System.out.printf("1: %d, 2: %d, 3: %d, 4: %d, 5: %d",
            m1, m2, m3, m4, m5);
        System.out.println();
    }

    private void runSwerveOdometry() {
      
        var gyroAngle = new Rotation2d(this.gyro.getFieldOrientation_rad());
    
        this.pose = 
          this.swerveDrivePoseEstimator.update(gyroAngle, 
            this.gSwerveModulePositions());
    }

    private SwerveModulePosition[] gSwerveModulePositions() {
        return new SwerveModulePosition[] {
            this.swerveModule1.getSwerveModulePosition(),
            this.swerveModule2.getSwerveModulePosition(),
            this.swerveModule3.getSwerveModulePosition(),
            this.swerveModule4.getSwerveModulePosition(),
            this.swerveModule5.getSwerveModulePosition(),
        };
    }


    public SwerveDriveKinematics getKinematics() {
        return this.swerveDriveKinematics;
    }

    public Pose2d getPose() {
        return this.pose;
    }
    
    public void zeroGyro() {
        gyro.resetOffsetToZero_rad();
        this.targetFieldOrientation_rad = 0.0;
    }

    public void enableGyro() {
        this.gyro.setEnabled(true);
    }

    public void disableGyro() {
        this.gyro.setEnabled(false);
    }

    public void executeAutonomousControl(
        Vector2D fieldOrientedTranslationCommand_in_s,
        Vector2D robotOrientedTranslationCommand_in_s,
        double robotOrientedRotationCommand_rad_s) {

        if(this.gyro.getEnabled()) {
            this.execute(
                fieldOrientedTranslationCommand_in_s, 
                robotOrientedTranslationCommand_in_s,
                robotOrientedRotationCommand_rad_s,
                false);
        }
        else {
            this.execute(Vector2D.FromXY(0, 0), Vector2D.FromXY(0, 0), 0.0, false);    
        }
    }
      
    public void executeAutonomousControlRobotOriented(
        Vector2D robotOrientedTranslationCommand_in_s,
        double robotOrientedRotationCommand_rad_s) {
        robotOrientedSwerve.execute(
            robotOrientedTranslationCommand_in_s, 
            robotOrientedRotationCommand_rad_s);
        runSwerveOdometry();
    }

    public void autonomousInit() {
        this.gyro.setEnabled(true);
        this.zeroGyro();
        this.fieldOrientedSwerve.setMaxAcceleration(Robot.MAX_ACCELERATION_AUTO_G*386.0);
        this.swerveDrivePoseEstimator.resetPosition(
            new Rotation2d(), 
            this.gSwerveModulePositions(), 
            new Pose2d(0.0, 0.0, new Rotation2d()));
    }

    public void teleopInit() {
        this.fieldOrientedSwerve.setMaxAcceleration(Robot.MAX_ACCELERATION_TELEOP_G*386.0);
        this.gyro.setEnabled(true);
        this.targetFieldOrientation_rad = this.gyro.getFieldOrientation_rad(); // prevents turning when you power up
    }
    
    public double getFieldOrientation_rad() {
        return this.gyro.getFieldOrientation_rad();
    }

    public void setGyroTo(double desiredOrientation_rad) {
        var currentAngle_rad = this.gyro.getFieldOrientation_rad();
        var adjustment_rad = currentAngle_rad - desiredOrientation_rad;
        this.gyro.adjustOffset_rad(adjustment_rad);
        this.targetFieldOrientation_rad = desiredOrientation_rad;
    }

    public void SetRobotFieldPosition(Pose2d pose, Rotation2d gyroAngle) {
        this.swerveDrivePoseEstimator.resetPosition(
            gyroAngle,
            this.gSwerveModulePositions(),
            pose
        );
    }
    public void setTargetFieldOrientation_rad(double targetFieldOrientation_rad) {
        this.targetFieldOrientation_rad = targetFieldOrientation_rad;
    }

    public void setLowCurrentLimit() {
        for (ISwerveModule swerveModule : this.swerveModules) {
            swerveModule.setLowCurrentLimit();
        }
    }

    public void setMaxCurrentLimit() {
        for (ISwerveModule swerveModule : this.swerveModules) {
            swerveModule.setMaxCurrentLimit();
        }
    }

    public void setRotationP(double rotationP) {
        this.fieldOrientedSwerve.setRotationP(rotationP);
    }

    public boolean isInsideCommunity() {
        return this.getPose().getX() <= 90.0;
    }

    public void setMaxRotationRate(double newMaxRotationRate_rad_s) {
        this.fieldOrientedSwerve.setMaxRotationRate(newMaxRotationRate_rad_s);
    }
}
