package frc.robot.subsystems.arm;

import com.ctre.phoenixpro.*;
import com.ctre.phoenixpro.configs.*;
import com.ctre.phoenixpro.controls.*;
import com.ctre.phoenixpro.hardware.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.communications.NetworkTableComms;

public class JointSubsystem extends SubsystemBase {
    private static final boolean INVERTED = false;
    private static final double REVERSE_HARD_STOP_ROT = -0.20;
    private static final double FORWARD_HARD_STOP_ROT = 17.50;

    private static final double CONE_SCORE_HIGH_POSITION_ROT = 17.30;
    private static final double CONE_SCORE_MID_POSITION_ROT = 15.00;

    private static final double CUBE_SCORE_HIGH_POSITION_ROT = 15.50;
    private static final double CUBE_SCORE_MID_POSITION_ROT = 12.00;

    private static final double PICK_POSITION_ROT = 0.03;
    public static final double POSITION_TOLERANCE_UP_ROT = 6.00; // 0.30
    private static final double POSITION_TOLERANCE_PICK_ROT = 0.30;

    private final TalonFX leftMotor = new TalonFX(41, "Default Name");
    private final TalonFX rightMotor = new TalonFX(42, "Default Name");
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0); // for simple open loop control
    private final MotionMagicTorqueCurrentFOC motionControl = new MotionMagicTorqueCurrentFOC(0, 0, 0, false);
    private final TorqueCurrentFOC torqueControl = new TorqueCurrentFOC(0);
    private final NeutralOut brake = new NeutralOut();
    private final Solenoid unlockSolenoid = new Solenoid(2, PneumaticsModuleType.REVPH, 4);

    private final NetworkTableComms nt;
    private final ArmHomeSensor armHomeSensor;
    private boolean homed;
    private double targetPosition_rot = PICK_POSITION_ROT;

    private TalonFXConfiguration leaderConfiguration;
    private GripperSubsystem gripperSubsystem;

    public JointSubsystem(
            NetworkTableComms nt,
            ArmHomeSensor armHomeSensor,
            GripperSubsystem gripperSubsystem) {
        
        this.nt = nt;
        this.armHomeSensor = armHomeSensor;
        this.gripperSubsystem = gripperSubsystem;

        this.configureMotor(this.leftMotor, false);
        this.configureMotor(this.rightMotor, true);

        this.leftMotor.setInverted(INVERTED); // call this before setting the position to zero in the next step, or it'll change the position
        this.leftMotor.setRotorPosition(0.0);

        this.rightMotor.setControl(new Follower(this.leftMotor.getDeviceID(), true));
    }

    private void configureMotor(TalonFX talon, boolean follower) {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        // Make sure we don't pop a breaker
        configs.CurrentLimits.SupplyCurrentLimit = 40;
        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        configs.CurrentLimits.StatorCurrentLimit = 60; 
        configs.CurrentLimits.StatorCurrentLimitEnable = true;

        if(!follower) {
            configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = REVERSE_HARD_STOP_ROT-0.05;
            configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_HARD_STOP_ROT+0.05;
            configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

            MotionMagicConfigs mm = new MotionMagicConfigs();
            mm.MotionMagicCruiseVelocity = 80; // ? rotations per second cruise -- note that 5000 RPM would be 83 rot/s
            mm.MotionMagicAcceleration = 175; // rot/s/s
            mm.MotionMagicJerk = 450; // rot/s/s/s
            configs.MotionMagic = mm;
    
            configs.Slot0.kP = 65; // An error of 1 rotations/sec results in ? amps output -- good value is 65
            configs.Slot0.kD = 4; // A change of 1 rotation per second results in ? amps output -- good value is 4
            configs.Slot0.kI = 0; 
            configs.Slot0.kV = 0.0; // feed forward (A)
            configs.Slot0.kS = 1; // A to get it moving
            
            // Peak output of ? amps (this is stator current)
            configs.TorqueCurrent.PeakForwardTorqueCurrent = 60;
            configs.TorqueCurrent.PeakReverseTorqueCurrent = -60;

            this.leaderConfiguration = configs;
        }

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = talon.getConfigurator().apply(configs);
            if (status.isOK()) break;
        }
        if(!status.isOK()) {
            System.out.println("Joint motor CAN " + talon.getDeviceID() + " could not apply configs, error code: " + status.toString());
        }
    }

    private void setSlow() {
        this.leaderConfiguration.MotionMagic.MotionMagicCruiseVelocity = 80.0;
        this.leaderConfiguration.MotionMagic.MotionMagicAcceleration = 125.0;
        this.leaderConfiguration.MotionMagic.MotionMagicJerk = 425.0;

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = this.leftMotor.getConfigurator().apply(this.leaderConfiguration);
            if (status.isOK()) break;
        }
        if(!status.isOK()) {
            System.out.println("Joint motor CAN " + this.leftMotor.getDeviceID() + " could not apply configs, error code: " + status.toString());
        }
    }

    private void setMedium() {
        this.leaderConfiguration.MotionMagic.MotionMagicCruiseVelocity = 80.0;
        this.leaderConfiguration.MotionMagic.MotionMagicAcceleration = 325.0;
        this.leaderConfiguration.MotionMagic.MotionMagicJerk = 800.0;

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = this.leftMotor.getConfigurator().apply(this.leaderConfiguration);
            if (status.isOK()) break;
        }
        if(!status.isOK()) {
            System.out.println("Joint motor CAN " + this.leftMotor.getDeviceID() + " could not apply configs, error code: " + status.toString());
        }
    }

    private void setFast() {
        this.leaderConfiguration.MotionMagic.MotionMagicCruiseVelocity = 85.0;
        this.leaderConfiguration.MotionMagic.MotionMagicAcceleration = 450.0;
        this.leaderConfiguration.MotionMagic.MotionMagicJerk = 3000.0;

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = this.leftMotor.getConfigurator().apply(this.leaderConfiguration);
            if (status.isOK()) break;
        }
        if(!status.isOK()) {
            System.out.println("Joint motor CAN " + this.leftMotor.getDeviceID() + " could not apply configs, error code: " + status.toString());
        }
    }

    public void robotPeriodic() {
        var position_rot = this.leftMotor.getPosition();
        if(!position_rot.getError().isOK()) {
            this.homed = false;
        }

        if(!this.homed) {
            if(this.armHomeSensor.atHome() && this.leftMotor.isAlive()) {
                this.targetPosition_rot = PICK_POSITION_ROT;
                var status = this.leftMotor.setRotorPosition(0.0);
                if(status.isOK()) {
                    this.homed = true;
                }
            }
        }

        this.nt.setJointHomed(this.homed);
        this.nt.setJointPosition(position_rot.getValue());
    }

    public boolean isHomed() {
        return this.homed;
    }

    @Override
    public void periodic() {

        this.unlockSolenoid.set(this.homed); // turns off when disabled or not homed, which engages the ratchet

        if(this.homed) {
            this.move();
        }
        else {
            this.leftMotor.setControl(
                new DutyCycleOut(0.0));
        }

        this.gripperSubsystem.SetJointVelocity_rps(this.getArmVelocity_rps());
        this.gripperSubsystem.SetJointPosition_rot(this.getArmPosition_rot());
    }

    private void move() {
        if(this.targetIsPick() && this.isAtPick()) {
            // drive into hard stop, value is in amps, negative direction is into hard stop
            var holdingTorque = -8;
            if(!this.nt.getExtensionAtPick()) {
                holdingTorque = -20; // keep the arm holding while the carriage is moving up/down because it's not locked in
            }

            this.leftMotor.setControl(
                this.torqueControl
                .withOutput(holdingTorque)); 
        }
        else {
            this.moveWithCounterbalance();
        }
        // System.out.printf("Joint: %.2f rot, M1: %.2f A, ", this.getPosition_rot(), this.getTorqueCurrent_A());
        // System.out.printf("M2: %.2f A", this.rightMotor.getStatorCurrent().getValue());
        // System.out.println();
    }

    private void moveWithCounterbalance() {
        var counterbalanceRatio = this.getJointCounterbalanceRatio();
        this.leftMotor.setControl(
            this.motionControl
                .withFeedForward(counterbalanceRatio * 6.2)
                .withPosition(this.targetPosition_rot));
    }

    public double getJointCounterbalanceRatio() {
        var armAngle_rad = getArmAngle_rad();
        var counterbalanceRatio = Math.sin(armAngle_rad); // as the joint goes up, we need more joint counterbalance
        return counterbalanceRatio;
    }

    public double getArmAngle_rad() {
        var armAngle_deg = 360.0 * this.getArmPosition_rot() / 16.0 + 12.0; // resting arm position is about 12 degrees
        var armAngle_rad = Math.toRadians(armAngle_deg);
        return armAngle_rad;
    }

    public double getArmPosition_rot() {
        return this.getPosition_rot() / this.getRatio();
    }

    public double getArmVelocity_rps() {
        return this.getVelocity_rps() / this.getRatio();
    }

    private double getRatio() {
        // 16 to 1 gearbox, 4 to 1 sprocket ratio
        return 4.0 * 16.0;
    }

    public void execute(double speedZeroToOne) {
        this.leftMotor.setControl(this.dutyCycleOut.withOutput(speedZeroToOne));
    }

    public void Brake() {
        this.leftMotor.setControl(this.brake);
    }

    public double getVelocity_rps() {
        return this.leftMotor.getVelocity().getValue();
    }

    public double getSupplyCurrent_A() {
        return this.leftMotor.getSupplyCurrent().getValue();
    }

    public double getTorqueCurrent_A() {
        return this.leftMotor.getTorqueCurrent().getValue();
    }

    public double getPosition_rot() {
        return this.leftMotor.getPosition().getValue();
    }

    public double MoveToScoreHigh() {
        this.setMedium();
        var position_rot = this.scoreHighPosition();
        this.MoveToPos(position_rot);
        return position_rot;
    }

    private double scoreHighPosition() {
        if(this.nt.isGamepieceCube()) {
            return CUBE_SCORE_HIGH_POSITION_ROT;
        }
        return CONE_SCORE_HIGH_POSITION_ROT;
    }

    public double MoveToScoreMid() {
        this.setSlow();
        var position_rot = this.scoreMidPosition();
        this.MoveToPos(position_rot);
        return position_rot;
    }

    private double scoreMidPosition() {
        if(this.nt.isGamepieceCube()) {
            return CUBE_SCORE_MID_POSITION_ROT;
        }
        return CONE_SCORE_MID_POSITION_ROT;
    }

    public void MoveToPick() {
        if(this.gripperSubsystem.isGripped()) {
            this.setSlow();
        }
        else {
            this.setFast();
        }
        this.MoveToPos(PICK_POSITION_ROT);
    }

    public boolean isNotAtPick() {
        return !this.isAtPick();
    }

    public boolean isAtPick() {
        return this.isAtPosition(PICK_POSITION_ROT, POSITION_TOLERANCE_PICK_ROT)
            || this.getPosition_rot() < PICK_POSITION_ROT;
    }

    public boolean isAtConePlace() {
        return this.isAtPosition(CONE_SCORE_HIGH_POSITION_ROT, POSITION_TOLERANCE_UP_ROT)
        || this.isAtPosition(CONE_SCORE_MID_POSITION_ROT, POSITION_TOLERANCE_UP_ROT);
    }

    public void MoveToPos(double pos_rot) {
        this.targetPosition_rot = pos_rot;
    }

    public boolean isAtPosition(double position_rot, double tolerance_rot) {
        var diff_rot = Math.abs(this.getPosition_rot() - position_rot);
        return diff_rot <= tolerance_rot;
    }

    public boolean targetIsPick() {
        return this.targetPosition_rot <= PICK_POSITION_ROT + 0.01;
    }
}
