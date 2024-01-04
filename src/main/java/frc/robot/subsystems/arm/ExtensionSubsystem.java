package frc.robot.subsystems.arm;

import com.ctre.phoenixpro.*;
import com.ctre.phoenixpro.configs.*;
import com.ctre.phoenixpro.controls.*;
import com.ctre.phoenixpro.hardware.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.communications.NetworkTableComms;
import frc.robot.utilities.OnDelayTimer;

public class ExtensionSubsystem extends SubsystemBase {
    private static final boolean INVERTED = false; // true;
    private static final double REVERSE_HARD_STOP_ROT = -11.0; // -2.40;
    private static final double FORWARD_HARD_STOP_ROT = 21.8; // 4.36;
    private static final double RETRACTED_POSITION_ROT = -10.6; // -2.32;
    private static final double PICK_POSITION_ROT = -0.1; // -0.02;

    private static final double CONE_SCORE_HIGH_POSITION_ROT = 21.5; // 3.60;
    private static final double CONE_SCORE_MID_POSITION_ROT = -2.15; // -0.73;

    private static final double CUBE_SCORE_HIGH_POSITION_ROT = 21.5; // 4.16;
    private static final double CUBE_SCORE_MID_POSITION_ROT = 2.5; // 0.50;

    private static final double POSITION_TOLERANCE_ROT = 5.0; // 0.50;
    private static final double POSITION_TOLERANCE_PRECISE_ROT = 0.5; 

    private static final double HOLD_RETRACTED_TORQUE_HIGH = 24; // 90; // stator amps - counteract centripetal force and robot moving while arm ext/retract
    private static final double HOLD_RETRACTED_TORQUE_LOW = 18; // 80 // stator amps - hold in arm hold position
    private static final double FEED_FORWARD_TORQUE = 15; // 65; // stator amps - counteract gravity

    private final TalonFX extensionMotor = new TalonFX(52, "Default Name");
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0); // for simple open loop control
    private final TorqueCurrentFOC torqueOut = new TorqueCurrentFOC(0);
    private final MotionMagicTorqueCurrentFOC motionControl = new MotionMagicTorqueCurrentFOC(0, 0, 0, false);
    private final JointSubsystem jointSubsystem;

    private final NetworkTableComms nt;
    private final ArmHomeSensor armHomeSensor;
    private final OnDelayTimer inHolsterTimer = new OnDelayTimer(250);
    private boolean homed;
    private double targetPosition_rot = PICK_POSITION_ROT;

    private static final double INCREMENT_ROT = 4.0 * Robot.SCAN_TIME_S; // how much to move target per robot scan

    private TalonFXConfiguration motorConfiguration;
    private Joystick joystick;

    public ExtensionSubsystem(
        JointSubsystem jointSubsystem,
        NetworkTableComms nt,
        ArmHomeSensor armHomeSensor, 
        Joystick joystick) {
        
        this.jointSubsystem = jointSubsystem;
        this.nt = nt;
        this.armHomeSensor = armHomeSensor;
        this.joystick = joystick;
        this.configureMotor(this.extensionMotor);
        this.extensionMotor.setInverted(INVERTED); // call this before setting the position to zero in the next step, or it'll change the position
        this.extensionMotor.setRotorPosition(0.0);
    }

    private void configureMotor(TalonFX talon) {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        // Make sure we don't pop a breaker
        configs.CurrentLimits.SupplyCurrentLimit = 40;
        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        configs.CurrentLimits.StatorCurrentLimit = 30; // needs roughly 30 to move it, 100 to lift (with no game piece), 125 with cone, don't want to drive past hard stop on end
        configs.CurrentLimits.StatorCurrentLimitEnable = true;
        configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = REVERSE_HARD_STOP_ROT-0.05;
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FORWARD_HARD_STOP_ROT+0.05;
        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        MotionMagicConfigs mm = new MotionMagicConfigs();
        mm.MotionMagicCruiseVelocity = 80; // ? rotations per second cruise -- note that 5000 RPM would be 83 rot/s -- 70 is good
        mm.MotionMagicAcceleration = 250; // 30; // rot/s/s -- 30 is good
        mm.MotionMagicJerk = 2500; // rot/s/s/s
        configs.MotionMagic = mm;

        configs.Slot0.kP = 55; // 350; // An error of 1 rotations/sec results in ? amps output
        configs.Slot0.kD = 0; // A change of 1 rotation per second results in ? amps output
        configs.Slot0.kI = 0.0; 
        configs.Slot0.kV = 0.0; // feed forward (A)
        configs.Slot0.kS = 0.0; // 25; // A to get it moving

        // Peak output in amps (stator current)
        configs.TorqueCurrent.PeakForwardTorqueCurrent = 25; // 80; // should be roughly 30 or less, 60 is a good guess
        configs.TorqueCurrent.PeakReverseTorqueCurrent = -50; // -180; // should be roughly 125 or less, -180 is a good guess

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = talon.getConfigurator().apply(configs);
            if (status.isOK()) break;
        }
        if(!status.isOK()) {
            System.out.println("Extension motor CAN " + talon.getDeviceID() + " could not apply configs, error code: " + status.toString());
        }

        this.motorConfiguration = configs;
    }

    private void setSlow() {
        this.motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = 80.0;
        this.motorConfiguration.MotionMagic.MotionMagicAcceleration = 250.0;
        this.motorConfiguration.MotionMagic.MotionMagicJerk = 2500.0;

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = this.extensionMotor.getConfigurator().apply(this.motorConfiguration);
            if (status.isOK()) break;
        }
        if(!status.isOK()) {
            System.out.println("Extension motor CAN " + this.extensionMotor.getDeviceID() + " could not apply configs, error code: " + status.toString());
        }
    }

    private void setFast() {
        this.motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = 80.0;
        this.motorConfiguration.MotionMagic.MotionMagicAcceleration = 500.0;
        this.motorConfiguration.MotionMagic.MotionMagicJerk = 5000.0;

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = this.extensionMotor.getConfigurator().apply(this.motorConfiguration);
            if (status.isOK()) break;
        }
        if(!status.isOK()) {
            System.out.println("Extension motor CAN " + this.extensionMotor.getDeviceID() + " could not apply configs, error code: " + status.toString());
        }
    }

    public void robotPeriodic() {
        var position_rot = this.extensionMotor.getPosition();
        if(!position_rot.getError().isOK()) {
            this.homed = false;
        }

        if(!this.homed) {
            if(this.armHomeSensor.atHome()) {
                this.homeMotor();
            }
        }

        this.nt.setExtensionHomed(this.homed);
        this.nt.setExtensionPosition(position_rot.getValue());
    }

    private void homeMotor() {
        this.targetPosition_rot = PICK_POSITION_ROT;
        var status = this.extensionMotor.setRotorPosition(0.0);
        if(status.isOK()) {
            this.homed = true;
        }
    }

    public boolean isHomed() {
        return this.homed;
    }

    @Override
    public void periodic() {
        // System.out.printf("Extension: %.2f, %.2f rot, %.2f A", this.targetPosition_rot, this.getPosition_rot(), this.getTorqueCurrent_A());
        // System.out.println();

        if(this.homed) {
            this.moveWithCounterbalance();
        }
        else {
            // turn off the motor if not homed
            this.extensionMotor.setControl(this.dutyCycleOut.withOutput(0.0));
        }

        this.nt.setExtensionAtPick(this.isAtPick());
    }

    private void moveWithCounterbalance() {
        var counterbalanceRatio = this.getExtensionCounterbalanceRatio();
        if(targetIsPick() && (armAtPick() || this.armHomeSensor.atHome())) {
            this.inHolsterTimer.execute(true);
            // turn off the motor if we're just resting in the holster
            this.extensionMotor.setControl(
                dutyCycleOut.withOutput(0)
            );

            var diff_rot = Math.abs(this.getPosition_rot());
            if(this.armHomeSensor.atHome() && this.inHolsterTimer.getOutput() && diff_rot > 0.1) {
                this.homeMotor(); // in case of belt slip
            }
        }
        else {
            this.inHolsterTimer.execute(false);
            if(this.targetIsRetract() && this.isRetractedPrecise()) {
                // we can lower torque while joint is holding against the stop because we're mostly just dealing with gravity
                var lowTorque = this.jointSubsystem.targetIsPick() && this.jointSubsystem.isAtPick();
                var torque_A = lowTorque ? HOLD_RETRACTED_TORQUE_LOW : HOLD_RETRACTED_TORQUE_HIGH;
                this.extensionMotor.setControl(
                    this.torqueOut.withOutput(-Math.abs(torque_A))
                );
            }
            else {
                var pov = this.joystick.getPOV();
                if(pov == 0) {
                    this.IncrementPosition();
                }
                else if(pov == 180) {
                    this.DecrementPosition();
                }
                this.extensionMotor.setControl(
                    this.motionControl
                        .withFeedForward(counterbalanceRatio * -Math.abs(FEED_FORWARD_TORQUE))
                        .withPosition(this.targetPosition_rot));
            }
        }
    }

    private boolean armAtPick() {
        return this.jointSubsystem.isAtPick() && this.isAtPick();
    }

    private boolean targetIsPick() {
        var diff_rot = Math.abs(this.targetPosition_rot - PICK_POSITION_ROT);
        return diff_rot <= POSITION_TOLERANCE_ROT;
    }

    private boolean targetIsRetract() {
        var diff_rot = Math.abs(this.targetPosition_rot - RETRACTED_POSITION_ROT);
        return diff_rot <= POSITION_TOLERANCE_ROT;
    }

    public void execute(double speedZeroToOne) {
        this.extensionMotor.setControl(this.dutyCycleOut.withOutput(speedZeroToOne));
    }

    public double getVelocity_rps() {
        return this.extensionMotor.getVelocity().getValue();
    }

    public double getSupplyCurrent_A() {
        return this.extensionMotor.getSupplyCurrent().getValue();
    }

    public double getTorqueCurrent_A() {
        return this.extensionMotor.getTorqueCurrent().getValue();
    }

    public double getPosition_rot() {
        return this.extensionMotor.getPosition().getValue();
    }

    public void MoveToRetract() {
        this.setFast();
        this.MoveToPos(RETRACTED_POSITION_ROT);
    }

    public boolean isRetracted() {
        return this.isAtPosition(RETRACTED_POSITION_ROT)
            || this.getPosition_rot() < RETRACTED_POSITION_ROT;
    }

    private boolean isRetractedPrecise() {
        return this.isAtPositionWithTolerance(RETRACTED_POSITION_ROT, POSITION_TOLERANCE_PRECISE_ROT)
            || this.getPosition_rot() < RETRACTED_POSITION_ROT;
    }

    public double MoveToScoreHigh() {
        this.setSlow();
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
        this.setSlow();
        this.MoveToPos(PICK_POSITION_ROT);
    }

    public boolean isAtPick() {
        return this.isAtPosition(PICK_POSITION_ROT);
    }

    public boolean isAtConePlace() {
        return this.isAtPosition(CONE_SCORE_HIGH_POSITION_ROT)
        || this.isAtPosition(CONE_SCORE_MID_POSITION_ROT);
    }

    public void MoveToPos(double pos_rot) {
        this.targetPosition_rot = pos_rot;
    }

    public boolean isAtPosition(double position_rot) {
        return this.isAtPositionWithTolerance(position_rot, POSITION_TOLERANCE_ROT);
    }

    private boolean isAtPositionWithTolerance(double position_rot, double positionTolerance_rot) {
        var diff_rot = Math.abs(this.getPosition_rot() - position_rot);
        return diff_rot <= positionTolerance_rot;
    }

    public double getExtensionCounterbalanceRatio() {
        var armAngle_rad = this.jointSubsystem.getArmAngle_rad();
        var counterbalanceRatio = Math.cos(armAngle_rad); // as the joint goes up, we need less extension counterbalance
        return counterbalanceRatio;
    }

    public void IncrementPosition() {
        if(this.jointSubsystem.isNotAtPick() && this.targetPosition_rot < FORWARD_HARD_STOP_ROT - INCREMENT_ROT) {
            this.targetPosition_rot += INCREMENT_ROT;
        }
    }

    public void DecrementPosition() {
        if(this.jointSubsystem.isNotAtPick() && this.targetPosition_rot > REVERSE_HARD_STOP_ROT + INCREMENT_ROT) {
            this.targetPosition_rot -= INCREMENT_ROT;
        }
    }
}
